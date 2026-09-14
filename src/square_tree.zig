const std = @import("std");
const calc = @import("calc.zig");
const index = @import("index.zig");
const para = @import("parallel.zig");
const svg = @import("svg.zig");
const vol = @import("volume.zig");
const math = std.math;
const Allocator = std.mem.Allocator;
const Io = std.Io;
const Vec2f = calc.Vec2f;
const Ball2f = vol.Ball2f;
const Box2f = vol.Box2f;
const OrientedBox2f = vol.OrientedBox2f;

/// A data structure that stores volumes + client IDs within an indexed region.
pub fn SquareTree(
    comptime Indexer: type, // Indexer used to structure tree.
    comptime Volume: type, // Type of volumes stored in leaf nodes.
    comptime ClientId: type, // Caller-chosen ID type.
) type {
    return struct {
        indexer: Indexer,
        num_volumes: usize = 0, // number of volumes currently stored
        max_half_extent: Vec2f = calc.zero2f, // largest half-extent of any stored volume
        bounds_valid: bool = false, // false if bounds need to be updated
        node_bvs: [depth][]Box2f, // BVs for all nodes
        leaf_data: []Volume, // all volumes, sorted by leaf number
        leaf_ids: []ClientId, // client ids for leaf_data, in the same order
        staged_data: []Volume, // volumes in insertion order, unsorted
        staged_ids: []ClientId, // client ids for staged_data, in the same order
        leaf_starts: []StartIndex, // stores the leaf_data start index for each leaf
        leaf_counts: []DataIndex, // the number of volumes within each leaf node
        scratch_a: []CurveIndex, // scratch buffer A - for BFS search
        scratch_b: []CurveIndex, // scratch buffer B - for BFS search
        max_async_workers: u16,

        /// Errors exposed through public methods:
        pub const Error = error{
            InputLengthMismatch,
            BufferCapacityExceeded,
            TreeCapacityExceeded,
            LeafCapacityExceeded,
            BoundsNotUpdated,
        };
        pub const ClientIdType = ClientId;
        pub const CurveIndex = Indexer.CurveIndex;
        pub const OverlapPair = [2]ClientIdType;
        pub const Neighbour = struct { id: ClientId, dist: f32 };
        pub const VolumeType = Volume;
        pub const compressed = Indexer.top_levels > 1;
        pub const depth = Indexer.depth;
        pub const nodes_in_level = Indexer.nodes_in_level;
        pub const num_leaves = Indexer.num_leaves;
        const DataIndex = u16; // Used to index volumes within leaf nodes.
        const StartIndex = u32; // Offset into leaf_data/leaf_ids
        const VolIndex = struct { leaf: CurveIndex, offset: DataIndex }; // locates a stored volume
        const max_parts_per_worker = 32;
        const max_ring_size: usize = 4 * math.sqrt(num_leaves);
        const worker_buf_bytes = 128 * 1024; // stack-allocated bytes for each worker
        const worker_buf_pair_len = worker_buf_bytes / @sizeOf(OverlapPair);
        const Self = @This();

        pub fn init(
            allocator: Allocator,
            bound_1: Vec2f, // a corner of the space to be covered
            bound_2: Vec2f, // the opposite corner of the space
            max_capacity: u32, // bounds the (heap-allocated) memory for storing volumes
            max_async_workers: u16, // limits the number of workers (0 defaults to cpu_count - 1)
        ) !Self {
            const indexer = try Indexer.init(bound_1, bound_2);
            const leaf_data = try allocator.alloc(Volume, max_capacity);
            errdefer allocator.free(leaf_data);
            const leaf_ids = try allocator.alloc(ClientId, max_capacity);
            errdefer allocator.free(leaf_ids);
            const leaf_starts = try allocator.alloc(StartIndex, num_leaves);
            errdefer allocator.free(leaf_starts);
            @memset(leaf_starts, 0);
            const leaf_counts = try allocator.alloc(DataIndex, num_leaves);
            errdefer allocator.free(leaf_counts);
            @memset(leaf_counts, 0);
            const staged_data = try allocator.alloc(Volume, max_capacity);
            errdefer allocator.free(staged_data);
            const staged_ids = try allocator.alloc(ClientId, max_capacity);
            errdefer allocator.free(staged_ids);
            var node_bvs: [depth][]Box2f = undefined;
            var levels_allocated: usize = 0;
            errdefer for (node_bvs[0..levels_allocated]) |s| allocator.free(s);
            inline for (0..depth) |lvl| {
                node_bvs[lvl] = try allocator.alloc(Box2f, nodes_in_level[lvl]);
                levels_allocated += 1;
            }
            // scratch buffer scales with number of workers (will be divided in parallel methods)
            const t: u16 = @truncate(@max(1, (try std.Thread.getCpuCount()) -| 1));
            const max_workers = if (max_async_workers > 0) @min(max_async_workers, t) else t;
            const scratch_buf_a = try allocator.alloc(CurveIndex, max_workers * num_leaves);
            errdefer allocator.free(scratch_buf_a);
            const scratch_buf_b = try allocator.alloc(CurveIndex, max_workers * num_leaves);
            errdefer allocator.free(scratch_buf_b);
            return Self{
                .indexer = indexer,
                .node_bvs = node_bvs,
                .leaf_data = leaf_data,
                .leaf_ids = leaf_ids,
                .leaf_starts = leaf_starts,
                .leaf_counts = leaf_counts,
                .staged_data = staged_data,
                .staged_ids = staged_ids,
                .scratch_a = scratch_buf_a,
                .scratch_b = scratch_buf_b,
                .max_async_workers = max_workers,
            };
        }

        pub fn deinit(self: *Self, allocator: Allocator) void {
            for (self.node_bvs) |v| allocator.free(v);
            allocator.free(self.scratch_a);
            allocator.free(self.scratch_b);
            allocator.free(self.staged_ids);
            allocator.free(self.staged_data);
            allocator.free(self.leaf_counts);
            allocator.free(self.leaf_starts);
            allocator.free(self.leaf_ids);
            allocator.free(self.leaf_data);
        }

        /// Adds volumes to the grid and stores their associated client ids (order must match).
        /// The volumes are staged: call `updateBounds` before querying.
        pub fn addVolumes(
            self: *Self,
            vols: []const Volume,
            client_ids: []const ClientId,
        ) Error!void {
            if (self.num_volumes + vols.len > self.staged_data.len) return error.TreeCapacityExceeded;
            if (vols.len != client_ids.len) return error.InputLengthMismatch;
            for (vols, client_ids) |v, c| {
                const leaf_num = self.indexer.getLeafIndexForPoint(v.getCentre());
                const data_index = self.leaf_counts[leaf_num];
                if (data_index == math.maxInt(DataIndex)) return error.LeafCapacityExceeded;
                if (compressed) {
                    const bb = v.getBoundingBox();
                    const he = calc.scaledVec(0.5, bb.max - bb.min);
                    self.max_half_extent = @max(self.max_half_extent, he);
                }
                self.staged_data[self.num_volumes] = v;
                self.staged_ids[self.num_volumes] = c;
                self.leaf_counts[leaf_num] = data_index + 1;
                self.num_volumes += 1;
            }
            self.bounds_valid = false;
        }

        /// Removes all volumes stored in leaf-nodes of the grid.
        pub fn clearStoredVolumes(self: *Self) void {
            @memset(self.leaf_counts, 0);
            self.bounds_valid = false;
            self.max_half_extent = calc.zero2f;
            self.num_volumes = 0;
        }

        /// Diagnostic: the largest number of volumes staged in any single leaf.
        pub fn getMaxLeafOccupancy(self: *const Self) DataIndex {
            var max_count: DataIndex = 0;
            for (self.leaf_counts) |count| max_count = @max(max_count, count);
            return max_count;
        }

        /// Gets the total number of volumes stored under a node in this tree's hierarchy.
        /// Counts the volumes stored under all successors' leaves.
        pub fn getOccupancyUnderNode(self: *const Self, lvl: u4, node: CurveIndex) usize {
            std.debug.assert(lvl < depth);
            const succ_start = Indexer.getFirstLeafSuccessor(@truncate(lvl), node);
            const succ_end = succ_start + Indexer.getNumberLeafSuccessors(@truncate(lvl));
            var total: usize = 0;
            for (succ_start..succ_end) |i| total += self.leaf_counts[i];
            return total;
        }

        /// Relocates the tree to a new position.
        /// Tree must be empty: call `clearStoredVolumes` first.
        pub fn relocate(self: *Self, new_min: Vec2f, new_max: Vec2f) !void {
            if (self.num_volumes > 0) return error.CannotRelocateOccupiedTree;
            self.indexer = try Indexer.init(new_min, new_max);
        }

        /// Grows all bounding volumes to cover all points within them.
        /// Sorts any volumes staged by `addVolume` into their final position.
        pub fn updateBounds(self: *Self) void {
            self.sortStagedVolumes();
            // start with leaf nodes first
            var range_iter = para.AtomicRangeIter.init(0, num_leaves, 1);
            self.updateLeafBvsWorker(self.node_bvs[depth - 1], &range_iter);
            // parent nodes, working up from the level above the leaves
            for (2..depth + 1) |i| {
                const lvl = depth - i;
                const child_bvs = self.node_bvs[lvl + 1];
                for (self.node_bvs[lvl], 0..) |*bv, j| {
                    var box = vol.empty_box;
                    const first_child = Indexer.getFirstChild(@truncate(j));
                    for (child_bvs[first_child..][0..Indexer.num_children]) |c| {
                        box = vol.getEncompassingBox(box, c);
                    }
                    bv.* = box;
                }
            }
            self.bounds_valid = true;
        }

        pub fn updateBoundsParallel(self: *Self, io: Io) !void {
            self.sortStagedVolumes(); // TODO: add a parallel version of this
            const parts = math.clamp(num_leaves / 8, 1, max_parts_per_worker * self.max_async_workers);
            const workers = math.clamp(parts / max_parts_per_worker, 1, self.max_async_workers);
            var range_iter = para.AtomicRangeIter.init(0, num_leaves, parts);
            var group: Io.Group = .init;
            errdefer group.cancel(io);
            const leaf_bvs = self.node_bvs[depth - 1];
            for (0..workers) |_| group.async(io, updateLeafBvsWorker, .{ self, leaf_bvs, &range_iter });
            try group.await(io);
            // parent nodes, working up from the level above the leaves
            for (2..depth + 1) |i| {
                const lvl = depth - i;
                const child_bvs = self.node_bvs[lvl + 1];
                for (self.node_bvs[lvl], 0..) |*bv, j| {
                    var box = vol.empty_box;
                    const first_child = Indexer.getFirstChild(@truncate(j));
                    for (child_bvs[first_child..][0..Indexer.num_children]) |c| {
                        box = vol.getEncompassingBox(box, c);
                    }
                    bv.* = box;
                }
            }
            self.bounds_valid = true;
        }

        fn updateLeafBvsWorker(self: *const Self, leaf_bvs: []Box2f, range_iter: *para.AtomicRangeIter) void {
            while (range_iter.next()) |range| {
                for (range.start..range.end) |i| {
                    const leaf_start = self.leaf_starts[i];
                    const leaf_end = leaf_start + self.leaf_counts[i];
                    var box = vol.empty_box;
                    for (self.leaf_data[leaf_start..leaf_end]) |v| box = vol.getEncompassingBox(box, v);
                    leaf_bvs[i] = box;
                }
            }
        }

        /// Returns id pairs for stored volumes that overlap with the provided query volumes.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        /// Single-threaded (no io dependency) but not thread-safe (writes to scratch bufs).
        pub fn findExtOverlaps(
            self: *Self,
            overlap_buf: []OverlapPair,
            query_ids: []const ClientId,
            query_vols: anytype,
        ) Error![]OverlapPair {
            if (query_ids.len != query_vols.len) return error.InputLengthMismatch;
            if (!self.bounds_valid) return error.BoundsNotUpdated;
            var range_iter = para.AtomicRangeIter.init(0, query_ids.len, 1);
            var out_counter: para.AtomicCounter = .{};
            self.findExtOverlapsWorker(
                self.scratch_a[0..num_leaves],
                self.scratch_b[0..num_leaves],
                &range_iter,
                &out_counter,
                overlap_buf,
                query_ids,
                query_vols,
            );
            return para.getOutputSlice(OverlapPair, &out_counter, overlap_buf);
        }

        /// Returns id pairs for stored volumes that overlap with the provided query volumes.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        /// Does work in parallel if the io implementation supports it; not thread-safe.
        pub fn findExtOverlapsParallel(
            self: *Self,
            io: Io,
            overlap_buf: []OverlapPair,
            query_ids: []const ClientId,
            query_vols: anytype,
        ) ![]OverlapPair {
            if (query_ids.len != query_vols.len) return error.InputLengthMismatch;
            if (!self.bounds_valid) return error.BoundsNotUpdated;
            if (query_ids.len == 0) return overlap_buf[0..0];
            // TODO: see if there's a way to remove the below struct?
            // This is a workaround since async won't allow a function with an anytype param.
            const Worker = struct {
                fn run(
                    tree: *const Self,
                    scratch_a: []CurveIndex,
                    scratch_b: []CurveIndex,
                    range_iter: *para.AtomicRangeIter,
                    out_counter: *para.AtomicCounter,
                    output: []OverlapPair,
                    ids: []const ClientId,
                    vols: @TypeOf(query_vols),
                ) void {
                    tree.findExtOverlapsWorker(scratch_a, scratch_b, range_iter, out_counter, output, ids, vols);
                }
            };
            const num_workers = @min(query_ids.len, self.max_async_workers);
            const num_parts = @max(1, @min(query_ids.len / 16, max_parts_per_worker * num_workers));
            var range_iter = para.AtomicRangeIter.init(0, query_ids.len, num_parts);
            var out_counter: para.AtomicCounter = .{};
            var group: Io.Group = .init;
            errdefer group.cancel(io);
            for (0..num_workers) |i| {
                group.async(io, Worker.run, .{
                    self,
                    self.scratch_a[i * num_leaves ..][0..num_leaves],
                    self.scratch_b[i * num_leaves ..][0..num_leaves],
                    &range_iter,
                    &out_counter,
                    overlap_buf,
                    query_ids,
                    query_vols,
                });
            }
            try group.await(io);
            return para.getOutputSlice(OverlapPair, &out_counter, overlap_buf);
        }

        fn findExtOverlapsWorker(
            self: *const Self,
            scratch_a: []CurveIndex,
            scratch_b: []CurveIndex,
            range_iter: *para.AtomicRangeIter,
            out_counter: *para.AtomicCounter,
            output: []OverlapPair,
            query_ids: []const ClientId,
            query_vols: anytype,
        ) void {
            var pair_buf: [worker_buf_pair_len]OverlapPair = undefined;
            var res_list = std.ArrayList(OverlapPair).initBuffer(&pair_buf);
            while (range_iter.next()) |range| {
                for (query_ids[range.start..range.end], query_vols[range.start..range.end]) |id, v| {
                    self.findOverlapsBfs(&res_list, scratch_a, scratch_b, id, v, 0, 0) catch
                        return reportStagingOverflow(out_counter, output);
                    para.copyToSharedBuffer(OverlapPair, &res_list, out_counter, output);
                }
            }
        }

        /// TODO: replace this with something that can return a more useful, descriptive error.
        fn reportStagingOverflow(out_counter: *para.AtomicCounter, output: []OverlapPair) void {
            _ = out_counter.value.fetchAdd(output.len + 1, .monotonic);
        }

        /// Returns ids for every pair of stored volumes that overlap with each other.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        /// Single-threaded (no io dependency) but not thread-safe (writes to scratch bufs).
        pub fn findSelfOverlaps(self: *Self, overlap_buf: []OverlapPair) Error![]OverlapPair {
            if (!self.bounds_valid) return error.BoundsNotUpdated;
            var range_iter = para.AtomicRangeIter.init(0, self.num_volumes, 1);
            var out_counter: para.AtomicCounter = .{};
            self.findSelfOverlapWorker(
                self.scratch_a[0..num_leaves],
                self.scratch_b[0..num_leaves],
                &range_iter,
                &out_counter,
                overlap_buf,
            );
            return para.getOutputSlice(OverlapPair, &out_counter, overlap_buf);
        }

        /// Returns ids for every pair of stored volumes that overlap with each other.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        /// Does work in parallel if the io implementation supports it; not thread-safe.
        pub fn findSelfOverlapsParallel(
            self: *Self,
            io: Io,
            overlap_buf: []OverlapPair,
        ) ![]OverlapPair {
            if (!self.bounds_valid) return error.BoundsNotUpdated;
            const num_volumes = self.num_volumes;
            if (num_volumes == 0) return overlap_buf[0..0];
            const num_workers = @min(num_volumes, self.max_async_workers);
            // aim for >= 32 volumes per partition, but never fewer than one partition
            const num_parts = math.clamp(num_volumes / 32, 1, max_parts_per_worker * num_workers);
            var range_iter = para.AtomicRangeIter.init(0, num_volumes, num_parts);
            var out_counter: para.AtomicCounter = .{};
            var group: Io.Group = .init;
            errdefer group.cancel(io);
            for (0..num_workers) |i| {
                group.async(io, findSelfOverlapWorker, .{
                    self,
                    self.scratch_a[i * num_leaves ..][0..num_leaves],
                    self.scratch_b[i * num_leaves ..][0..num_leaves],
                    &range_iter,
                    &out_counter,
                    overlap_buf,
                });
            }
            try group.await(io);
            return para.getOutputSlice(OverlapPair, &out_counter, overlap_buf);
        }

        fn findSelfOverlapWorker(
            self: *const Self,
            scratch_a: []CurveIndex,
            scratch_b: []CurveIndex,
            range_iter: *para.AtomicRangeIter,
            out_counter: *para.AtomicCounter,
            output: []OverlapPair,
        ) void {
            var pair_buf: [worker_buf_pair_len]OverlapPair = undefined;
            var res_list = std.ArrayList(OverlapPair).initBuffer(&pair_buf);
            while (range_iter.next()) |range| {
                if (range.start >= range.end) continue;
                var leaf_cursor = self.flatIndexToLeafIndex(range.start);
                for (range.start..range.end) |i| {
                    const vol_index = self.nextVolIndex(&leaf_cursor, i);
                    self.findOverlapsBfs(
                        &res_list,
                        scratch_a,
                        scratch_b,
                        self.leaf_ids[i],
                        self.leaf_data[i],
                        vol_index.leaf,
                        vol_index.offset + 1,
                    ) catch return reportStagingOverflow(out_counter, output);
                    para.copyToSharedBuffer(OverlapPair, &res_list, out_counter, output);
                }
            }
        }

        /// Finds stored volumes nearest to each query point, nearest-first.
        /// Search stops when k volumes are found, or there are no more candidates within `max_dist`.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        pub fn findNeighbours(
            self: *const Self,
            bufs: [][]Neighbour,
            points: []const Vec2f,
            excl_ids: []const ?ClientId,
            k: u16,
            max_dist: f32,
        ) Error![][]Neighbour {
            if (bufs.len != points.len or bufs.len != excl_ids.len) return error.InputLengthMismatch;
            if (!self.bounds_valid) return error.BoundsNotUpdated;
            for (bufs) |buf| if (k > buf.len) return error.BufferCapacityExceeded;
            var range_iter = para.AtomicRangeIter.init(0, bufs.len, 1);
            self.findNeighboursWorker(bufs, points, excl_ids, k, max_dist, &range_iter);
            return bufs;
        }

        /// Finds stored volumes nearest to the query point, nearest-first.
        /// Search stops when k volumes are found, or there are no more candidates within `max_dist`.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        /// Does work in parallel if the io implementation supports it; thread-safe.
        pub fn findNeighboursParallel(
            self: *const Self,
            io: Io,
            bufs: [][]Neighbour,
            points: []const Vec2f,
            excl_ids: []const ?ClientId,
            k: u16,
            max_dist: f32,
        ) ![][]Neighbour {
            if (bufs.len != points.len or bufs.len != excl_ids.len) return error.InputLengthMismatch;
            if (!self.bounds_valid) return error.BoundsNotUpdated;
            for (bufs) |buf| if (k > buf.len) return error.BufferCapacityExceeded;
            if (bufs.len == 0) return bufs;
            const num_workers = @min(bufs.len, self.max_async_workers);
            const num_parts = @min(bufs.len, 8 * num_workers);
            var range_iter = para.AtomicRangeIter.init(0, bufs.len, num_parts);
            var group: Io.Group = .init;
            errdefer group.cancel(io);
            for (0..num_workers) |_| {
                const args = .{ self, bufs, points, excl_ids, k, max_dist, &range_iter };
                group.async(io, findNeighboursWorker, args);
            }
            try group.await(io);
            return bufs;
        }

        /// Finds stored volumes nearest to a single query point, nearest-first.
        /// Search stops when k volumes are found, or there are no more candidates within `max_dist`.
        /// Requires `updateBounds` to have been called since the last `addVolume`.
        pub fn findNeighboursSingle(
            self: *const Self,
            buf: []Neighbour,
            point: Vec2f,
            excl_id: ?ClientId,
            k: u16,
            max_dist: f32,
        ) Error![]Neighbour {
            if (!self.bounds_valid) return error.BoundsNotUpdated;
            return self.neighboursForPoint(buf, point, excl_id, k, max_dist);
        }

        fn findNeighboursWorker(
            self: *const Self,
            bufs: [][]Neighbour,
            points: []const Vec2f,
            excl_ids: []const ?ClientId,
            k: u16,
            max_dist: f32,
            range_iter: *para.AtomicRangeIter,
        ) void {
            while (range_iter.next()) |range| {
                for (range.start..range.end) |i| {
                    bufs[i] = self.neighboursForPoint(
                        bufs[i],
                        points[i],
                        excl_ids[i],
                        k,
                        max_dist,
                    ) catch unreachable;
                }
            }
        }

        /// Draws a tree's grid subdivisions, cell labels, and stored volumes to an svg file.
        /// Accepts a pointer to any tree exposing the same public interface as `SquareTree`.
        pub fn drawTreeSvg(
            self: *Self,
            allocator: Allocator,
            show_client_ids: bool,
        ) !svg.Canvas {
            const bgs: svg.ShapeStyle = .{
                .fill_active = true,
                .fill_hsl = .{ 0, 0, 95 },
                .stroke_active = false,
            };
            var canvas = try svg.Canvas.init(allocator, self.indexer.min_pt, self.indexer.max_pt, bgs);
            errdefer canvas.deinit(allocator);
            const extent = self.indexer.max_pt - self.indexer.min_pt;
            const scale = @reduce(.Max, extent) / 1024.0;
            // draw grid subdivisions + cell labels, finest level first
            var palette = try svg.RandomHslPalette.init(allocator, depth, 0);
            defer palette.deinit(allocator);
            var label_buff: [16]u8 = undefined;
            for (calc.getReversedRange(Indexer.LevelIndex, depth)) |lvl| {
                const style: svg.ShapeStyle = .{
                    .stroke_hsl = palette.hsl_colours[lvl],
                    .stroke_width = scale * calc.asf32(depth - lvl),
                };
                const font_size: f32 = scale * (6.0 + 4.0 * calc.asf32(depth - lvl));
                for (0..nodes_in_level[lvl]) |i| {
                    const node_index: CurveIndex = @intCast(i);
                    const cell = self.indexer.getCellBoundaryAtLevel(lvl, node_index);
                    const label_width = (math.log2_int(usize, nodes_in_level[lvl]) + 3) / 4;
                    const label = try std.fmt.bufPrint(&label_buff, "{X:0>[1]}", .{ node_index, label_width });
                    try canvas.addRectangle(allocator, cell.min, cell.max, style);
                    try canvas.addText(allocator, cell.getCentre(), label, font_size, style.stroke_hsl);
                }
            }
            // find every client id that participates in an overlap
            const volumes = self.leaf_data[0..self.num_volumes];
            const ids = self.leaf_ids[0..self.num_volumes];
            const overlap_buff = try allocator.alloc(OverlapPair, 16 * volumes.len);
            defer allocator.free(overlap_buff);
            const pairs = try self.findSelfOverlaps(overlap_buff);
            var overlapping = std.AutoHashMap(ClientId, void).init(allocator);
            defer overlapping.deinit();
            for (pairs) |pair| {
                try overlapping.put(pair[0], {});
                try overlapping.put(pair[1], {});
            }
            //draw the stored volumes, colouring overlapping ones differently
            const default_style: svg.ShapeStyle = .{ .stroke_hsl = .{ 0, 0, 20 }, .stroke_width = scale * 1.0 };
            const overlap_style: svg.ShapeStyle = .{ .stroke_hsl = .{ 0, 80, 45 }, .stroke_width = scale * 2.0 };
            const id_label_hsl: [3]u9 = .{ 0, 0, 0 };
            const id_label_font_size: f32 = scale * 10.0;
            var id_buff: [20]u8 = undefined;
            for (volumes, ids) |v, id| {
                const style = if (overlapping.contains(id)) overlap_style else default_style;
                if (Volume == Ball2f) {
                    try canvas.addCircle(allocator, v.centre, v.radius, style);
                } else if (Volume == Box2f) {
                    try canvas.addRectangle(allocator, v.min, v.max, style);
                } else if (Volume == OrientedBox2f) {
                    var corners = v.getCorners();
                    try canvas.addPolygon(allocator, &corners, style);
                } else {
                    @compileError("drawTreeSvg: unsupported volume type " ++ @typeName(Volume));
                }
                if (show_client_ids) {
                    const label = try std.fmt.bufPrint(&id_buff, "{d}", .{id});
                    try canvas.addText(allocator, v.getCentre(), label, id_label_font_size, id_label_hsl);
                }
            }
            return canvas;
        }

        /// Gets the volumes stored in the specified leaf node, in insertion order.
        fn getLeafVolumes(self: *const Self, leaf_num: CurveIndex) []const Volume {
            const start = self.leaf_starts[leaf_num];
            return self.leaf_data[start..][0..self.leaf_counts[leaf_num]];
        }

        /// Gets the client ids stored in the specified leaf node, in the same order as getLeafVolumes.
        fn getLeafIds(self: *const Self, leaf_num: CurveIndex) []const ClientId {
            const start = self.leaf_starts[leaf_num];
            return self.leaf_ids[start..][0..self.leaf_counts[leaf_num]];
        }

        /// Sorts the staged volumes into leaf_data by leaf number and fills leaf_starts.
        fn sortStagedVolumes(self: *Self) void {
            // compute each leaf's start offset
            var offset: StartIndex = 0;
            for (self.leaf_starts, self.leaf_counts) |*start, count| {
                start.* = offset;
                offset += count;
            }
            // scatter, using leaf_starts as the per-leaf write cursor
            const num_vols = self.num_volumes;
            for (self.staged_data[0..num_vols], self.staged_ids[0..num_vols]) |v, id| {
                const leaf_num = self.indexer.getLeafIndexForPoint(v.getCentre());
                const write_index = self.leaf_starts[leaf_num];
                self.leaf_data[write_index] = v;
                self.leaf_ids[write_index] = id;
                self.leaf_starts[leaf_num] += 1;
            }
            // rewind the cursors, recovering the start offsets
            for (self.leaf_starts, self.leaf_counts) |*start, count| {
                start.* -= count;
            }
        }

        /// Finds the nearest neighbours for a single point; skips bounds check.
        fn neighboursForPoint(
            self: *const Self,
            buf: []Neighbour,
            point: Vec2f,
            exclude_id: ?ClientId,
            k: u16,
            max_dist: f32,
        ) Error![]Neighbour {
            if (k > buf.len) return error.BufferCapacityExceeded;
            const leaf_index = self.indexer.getLeafIndexForPoint(point);
            var len: usize = 0;
            var iter: u16 = 0;
            var furthest_dist: f32 = 0;
            var next_min_dist: f32 = 0;
            var scratch_buf: [max_ring_size]CurveIndex = undefined;
            while (next_min_dist < max_dist and (len < k or next_min_dist < furthest_dist)) {
                const leaves = try Indexer.getLeafCellNeighbours(&scratch_buf, leaf_index, iter);
                for (leaves) |i| {
                    const vols = self.getLeafVolumes(i);
                    for (vols, self.getLeafIds(i)) |v, client_id| {
                        if (exclude_id != null and exclude_id.? == client_id) continue;
                        const dist_squared = calc.squaredSum(v.getCentre() - point);
                        const threshold = if (len < k) max_dist else furthest_dist;
                        if (dist_squared >= threshold * threshold) continue;
                        const dist = @sqrt(dist_squared);
                        const n_info = Neighbour{ .id = client_id, .dist = dist };
                        var pos = if (len < k) len else k -| 1;
                        while (pos > 0 and buf[pos - 1].dist > dist) : (pos -= 1) {
                            buf[pos] = buf[pos - 1];
                        }
                        buf[pos] = n_info;
                        if (len < k) len += 1;
                        furthest_dist = buf[len - 1].dist;
                    }
                }
                next_min_dist = calc.asf32(iter) * self.indexer.cell_size;
                iter += 1;
            }
            return buf[0..len];
        }

        /// Does a binary search to find the leaf associated with a flat index (for leaf_data).
        fn flatIndexToLeafIndex(self: *const Self, flat_index: usize) CurveIndex {
            var lo: usize = 0;
            var hi: usize = num_leaves;
            while (lo + 1 < hi) {
                const mid = lo + (hi - lo) / 2;
                if (self.leaf_starts[mid] <= flat_index) lo = mid else hi = mid;
            }
            return @intCast(lo);
        }

        /// Returns the after-last index (in leaf_data) for the identified leaf node.
        fn leafEnd(self: *const Self, leaf_index: CurveIndex) usize {
            return @as(usize, self.leaf_starts[leaf_index]) + self.leaf_counts[leaf_index];
        }

        /// Increments the provided leaf index until it points to the leaf_data that includes flat_index.
        fn nextVolIndex(self: *const Self, leaf_idx_ptr: *CurveIndex, flat_index: usize) VolIndex {
            std.debug.assert(flat_index < self.num_volumes); // else the walk runs off the end
            while (flat_index >= self.leafEnd(leaf_idx_ptr.*)) leaf_idx_ptr.* += 1;
            return .{
                .leaf = @intCast(leaf_idx_ptr.*),
                .offset = @intCast(flat_index - self.leaf_starts[leaf_idx_ptr.*]),
            };
        }

        /// Performs a BFS for stored volumes that overlap with the provided query volume.
        fn findOverlapsBfs(
            self: *const Self,
            res_list: *std.ArrayList(OverlapPair),
            slice_a: []CurveIndex,
            slice_b: []CurveIndex,
            query_id: ClientId,
            query_vol: anytype,
            start_leaf: CurveIndex,
            start_vol_index: DataIndex,
        ) !void {
            // search through higher-level nodes first
            const query_aabb: Box2f = query_vol.getBoundingBox();
            var search_list = std.ArrayList(CurveIndex).initBuffer(slice_a);
            if (compressed) { // check the neighbouring level 0 nodes only
                const n_box: Box2f = .{
                    .min = query_aabb.min - self.max_half_extent,
                    .max = query_aabb.max + self.max_half_extent,
                };
                try self.indexer.getTopLevelIndexesForBox(&search_list, n_box, start_leaf);
            } else { // check all level 0 nodes
                const pred_0 = Indexer.getLeafPredecessor(start_leaf, 0);
                for (pred_0..nodes_in_level[0]) |k| try search_list.appendBounded(@intCast(k));
            }
            var next_list = std.ArrayList(CurveIndex).initBuffer(slice_b);
            for (0..depth - 1) |lvl| {
                const pred_next: usize = Indexer.getLeafPredecessor(start_leaf, @truncate(lvl + 1));
                for (search_list.items) |i| {
                    const node_vol = self.node_bvs[lvl][i];
                    if (!vol.checkVolumesOverlap(query_aabb, node_vol)) continue;
                    const first_child: usize = Indexer.getFirstChild(i);
                    const start = @max(pred_next, first_child);
                    const end = first_child + Indexer.num_children;
                    for (start..end) |k| try next_list.appendBounded(@intCast(k));
                }
                // Swap the buffers
                const tmp = search_list;
                search_list = next_list;
                next_list = tmp;
                next_list.clearRetainingCapacity();
            }
            // check the surviving leaf nodes for overlaps
            for (search_list.items) |i| {
                const leaf_vol = self.node_bvs[depth - 1][i];
                if (!vol.checkVolumesOverlap(query_aabb, leaf_vol)) continue;
                const items = self.getLeafVolumes(i);
                const ids = self.getLeafIds(i);
                const start = if (i == start_leaf) start_vol_index else 0;
                for (items[start..], ids[start..]) |stored_vol, id| {
                    if (!vol.checkVolumesOverlap(query_vol, stored_vol)) continue;
                    try res_list.appendBounded(.{ query_id, id });
                }
            }
        }
    };
}

const testing = std.testing;
const test_alloc = testing.allocator;
const test_capacity = 1000;

test "square tree init + deinit" {
    // check for memory leaks
    const Tree2x8 = SquareTree(index.Indexer2f(2, 1, 7, .Morton), Ball2f, u32);
    var qt = try Tree2x8.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 0);
    defer qt.deinit(test_alloc);
    const Tree4x4 = SquareTree(index.Indexer2f(4, 1, 5, .Zigzag), Ball2f, u32);
    var ht = try Tree4x4.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 0);
    defer ht.deinit(test_alloc);
}

test "hex tree overlap ball" {
    const HexTree2 = SquareTree(index.Indexer2f(4, 1, 1, .Zigzag), Ball2f, u32);
    var tree = try HexTree2.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 8);
    defer tree.deinit(test_alloc);
    var balls = [3]Ball2f{
        .{ .centre = .{ 0.2, 0.0 }, .radius = 0.4 },
        .{ .centre = .{ 0.2, 0.5 }, .radius = 0.2 },
        .{ .centre = .{ 0.2, 0.7 }, .radius = 0.1 },
    };
    const indexes = calc.getRange(u32, balls.len);
    try tree.addVolumes(balls[0..], &indexes);
    try tree.updateBoundsParallel(testing.io);
    var id_buff: [16][2]u32 = undefined;
    const query_ids = [_]u32{ 4, 5, 6 };
    const query_regions = [_]Ball2f{
        .{ .centre = .{ 0.9, 0.5 }, .radius = 0.1 },
        .{ .centre = .{ -3.5, -3.5 }, .radius = 1.0 },
        .{ .centre = .{ 0.2, 0.5 }, .radius = 0.2 },
    };
    // query balls 4 and 5 overlap nothing; 6 overlaps all 3 stored balls
    const ext_overlaps = try tree.findExtOverlapsParallel(
        testing.io,
        &id_buff,
        &query_ids,
        &query_regions,
    );
    const expected_ext = [_]HexTree2.OverlapPair{ .{ 0, 6 }, .{ 1, 6 }, .{ 2, 6 } };
    calc.sortPairsLessThan(u32, ext_overlaps);
    try testing.expectEqualSlices(HexTree2.OverlapPair, &expected_ext, ext_overlaps);
    // a overlaps b, and b overlaps c, but a does not overlap c.
    const self_overlaps = try tree.findSelfOverlapsParallel(testing.io, &id_buff);
    calc.sortPairsLessThan(u32, self_overlaps);
    const expected_self = [_]HexTree2.OverlapPair{ .{ 0, 1 }, .{ 1, 2 } };
    try testing.expectEqualSlices(HexTree2.OverlapPair, &expected_self, self_overlaps);
}

test "hex tree overlap box" {
    const HexTree2 = SquareTree(index.Indexer2f(4, 1, 1, .Zigzag), Box2f, u32);
    var tree = try HexTree2.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 0);
    defer tree.deinit(test_alloc);
    const boxes = [_]Box2f{
        .{ .min = .{ -0.2, -0.4 }, .max = .{ 0.6, 0.4 } },
        .{ .min = .{ 0.0, 0.3 }, .max = .{ 0.4, 0.7 } },
        .{ .min = .{ 0.1, 0.6 }, .max = .{ 0.3, 0.8 } },
    };

    const indexes = calc.getRange(u32, boxes.len);
    try tree.addVolumes(&boxes, &indexes);
    try tree.updateBoundsParallel(testing.io);
    const query_ids = [_]u32{ 4, 5, 6 };
    const query_regions = [_]Box2f{
        .{ .min = .{ 0.8, 0.4 }, .max = .{ 1.0, 0.6 } },
        .{ .min = @splat(-4.5), .max = @splat(-2.5) },
        .{ .min = .{ 0.0, 0.3 }, .max = .{ 0.4, 0.7 } },
    };
    // query boxes 4 and 5 overlap nothing; 6 overlaps all 3 stored boxes
    var id_buff: [16]HexTree2.OverlapPair = undefined;
    const ext_overlaps = try tree.findExtOverlaps(&id_buff, &query_ids, &query_regions);
    const expected_ext = [_]HexTree2.OverlapPair{ .{ 0, 6 }, .{ 1, 6 }, .{ 2, 6 } };
    calc.sortPairsLessThan(u32, ext_overlaps);
    try testing.expectEqualSlices(HexTree2.OverlapPair, &expected_ext, ext_overlaps);
    // a overlaps b, and b overlaps c, but a does not overlap c.
    const self_overlaps = try tree.findSelfOverlapsParallel(testing.io, &id_buff);
    calc.sortPairsLessThan(u32, self_overlaps);
    const expected_self = [_]HexTree2.OverlapPair{ .{ 0, 1 }, .{ 1, 2 } };
    try testing.expectEqualSlices(HexTree2.OverlapPair, &expected_self, self_overlaps);
}

test "square tree add remove" {
    const IndexerM2x4 = index.Indexer2f(4, 1, 1, .Morton);
    const QuadTree = SquareTree(IndexerM2x4, Ball2f, u32);
    var qt = try QuadTree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 0);
    defer qt.deinit(test_alloc);
    const test_bodies = [_]Ball2f{
        .{ .centre = .{ 0.2, 0.0 }, .radius = 0.4 },
        .{ .centre = .{ 0.2, 0.5 }, .radius = 0.2 },
        .{ .centre = .{ 0.2, 0.9 }, .radius = 0.1 },
    };
    const indexes = calc.getRange(u32, test_bodies.len);
    try qt.addVolumes(&test_bodies, &indexes);
    try testing.expectEqual(3, qt.num_volumes);
    qt.updateBounds();
    // check volumes retrieved by id come back unchanged
    for (0..QuadTree.num_leaves) |leaf_num_usize| {
        const leaf_num: QuadTree.CurveIndex = @intCast(leaf_num_usize);
        for (qt.getLeafVolumes(leaf_num), qt.getLeafIds(leaf_num)) |v, id| {
            try testing.expectEqual(test_bodies[id].centre, v.centre);
            try testing.expectEqual(test_bodies[id].radius, v.radius);
        }
    }
    qt.clearStoredVolumes();
    try testing.expectEqual(0, qt.num_volumes);
}

test "staged volumes keep their rank within a leaf" {
    const IndexerM4x2 = index.Indexer2f(4, 1, 1, .Morton);
    const QuadTree = SquareTree(IndexerM4x2, Ball2f, u32);
    var qt = try QuadTree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 0);
    defer qt.deinit(test_alloc);
    // interleave insertions between two distant leaves, so the scatter has to reorder
    var radii: [6]f32 = undefined;
    for (0..6) |i| {
        radii[i] = 0.01 * @as(f32, @floatFromInt(i + 1));
        const centre: Vec2f = if (i % 2 == 0) .{ 0.1, 0.1 } else .{ 0.9, 0.9 };
        const balls = [_]Ball2f{.{ .centre = centre, .radius = radii[i] }};
        const indexes = [_]u32{@intCast(i)};
        try qt.addVolumes(&balls, &indexes);
    }
    qt.updateBounds();
    // both leaves should see their volumes in insertion order
    const leaf_even = qt.indexer.getLeafIndexForPoint(.{ 0.1, 0.1 });
    const leaf_odd = qt.indexer.getLeafIndexForPoint(.{ 0.9, 0.9 });
    const even_vols = qt.getLeafVolumes(leaf_even);
    const odd_vols = qt.getLeafVolumes(leaf_odd);
    try testing.expectEqual(3, even_vols.len);
    try testing.expectEqual(3, odd_vols.len);
    for (even_vols, 0..) |v, rank| try testing.expectEqual(radii[rank * 2], v.radius);
    for (odd_vols, 0..) |v, rank| try testing.expectEqual(radii[rank * 2 + 1], v.radius);
}

test "find self overlaps matches brute force" {
    const Trees = .{
        SquareTree(index.Indexer2f(4, 1, 2, .Spring), Ball2f, u16),
        SquareTree(index.Indexer2f(4, 1, 1, .Zigzag), Box2f, u16),
        SquareTree(index.Indexer2f(2, 1, 4, .Morton), OrientedBox2f, u16),
        SquareTree(index.Indexer2f(4, 2, 1, .Spring), Ball2f, u16),
        SquareTree(index.Indexer2f(4, 2, 1, .Zigzag), OrientedBox2f, u16),
        SquareTree(index.Indexer2f(2, 3, 2, .Morton), Box2f, u16),
        SquareTree(index.Indexer2f(4, 3, 0, .Morton), Ball2f, u16),
        SquareTree(index.Indexer2f(2, 1, 5, .Morton), OrientedBox2f, u16),
        SquareTree(index.Indexer2f(2, 6, 0, .Morton), Box2f, u16),
    };
    const num_vols = 200;
    const seed = calc.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer calc.printErrorMessageForRandomSeed(seed);
    var test_vols = try vol.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_vols,
        .{ .uniform = .{ .min = 0.005, .max = 0.04 } },
        .{ .uniform = .{ .min = 0.05, .max = 0.95 } },
    );
    defer test_vols.deinit(test_alloc);
    // generate random volumes and check for overlaps between all pairs
    inline for (Trees) |Tree| {
        const bodies = test_vols.getRandomBodies(Tree.VolumeType);
        var tree = try Tree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, num_vols, 0);
        defer tree.deinit(test_alloc);
        const indexes = calc.getRange(Tree.ClientIdType, num_vols);
        try tree.addVolumes(bodies, &indexes);
        try tree.updateBoundsParallel(testing.io);
        var expected: std.ArrayList(Tree.OverlapPair) = .empty;
        defer expected.deinit(test_alloc);
        for (bodies, 0..) |a, i| {
            for (bodies[i + 1 ..], i + 1..) |b, j| {
                if (vol.checkVolumesOverlap(a, b)) {
                    try expected.append(test_alloc, .{ @intCast(i), @intCast(j) });
                }
            }
        }
        // check that the pairwise overlap results agree with those returned by the tree's method
        const found_buff = try test_alloc.alloc(Tree.OverlapPair, num_vols * num_vols);
        defer test_alloc.free(found_buff);
        const actual = try tree.findSelfOverlapsParallel(testing.io, found_buff);
        calc.sortPairsLessThan(Tree.ClientIdType, expected.items);
        calc.sortPairsLessThan(Tree.ClientIdType, actual);
        try testing.expectEqualSlices(Tree.OverlapPair, expected.items, actual);
    }
}

test "short overlap buffer returns a capacity error" {
    const Tree = SquareTree(index.Indexer2f(4, 1, 1, .Zigzag), Ball2f, u32);
    var tree = try Tree.init(test_alloc, .{ -1, -1 }, .{ 1, 1 }, 16, 0);
    defer tree.deinit(test_alloc);
    const balls = [_]Ball2f{.{ .centre = .{ 0, 0 }, .radius = 0.5 }} ** 16;
    const ids = calc.getRange(u32, balls.len);
    try tree.addVolumes(&balls, &ids);
    tree.updateBounds();
    var buf: [32]Tree.OverlapPair = undefined;
    try testing.expectError(error.BufferCapacityExceeded, tree.findSelfOverlaps(buf[0..4]));
    try testing.expectError(
        error.BufferCapacityExceeded,
        tree.findSelfOverlapsParallel(testing.io, buf[0..4]),
    );
}

test "find neighbours matches brute force" {
    const num_vols = 200;
    const seed = calc.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer calc.printErrorMessageForRandomSeed(seed);
    const Tree = SquareTree(index.Indexer2f(4, 2, 1, .Zigzag), Box2f, u16);
    var tree = try Tree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, num_vols, 0);
    defer tree.deinit(test_alloc);
    var test_vols = try vol.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_vols,
        .{ .uniform = .{ .min = 0.005, .max = 0.04 } },
        .{ .uniform = .{ .min = 0.05, .max = 0.95 } },
    );
    defer test_vols.deinit(test_alloc);
    const boxes = test_vols.getRandomBodies(Box2f);
    const indexes = calc.getRange(Tree.ClientIdType, num_vols);
    try tree.addVolumes(boxes, &indexes);
    tree.updateBounds();
    // check for closest neighbours between all pairs
    var expected: [num_vols][3]Tree.Neighbour = undefined;
    var neighbour_storage: [num_vols][3]Tree.Neighbour = undefined;
    var bufs: [num_vols][]Tree.Neighbour = undefined;
    var points: [num_vols]Vec2f = undefined;
    var excl_ids: [num_vols]?Tree.ClientIdType = undefined;
    for (boxes, indexes, 0..) |box_a, id_a, i| {
        var nearest = ([1]Tree.Neighbour{.{ .id = 0, .dist = math.floatMax(f32) }}) ** 3;
        const a_centre = box_a.getCentre();
        for (boxes, indexes) |box_b, id_b| {
            if (id_a == id_b) continue;
            const b_dist = calc.norm(box_b.getCentre() - a_centre);
            for (0..3) |rank| {
                if (b_dist < nearest[rank].dist) {
                    var j: usize = 2;
                    while (j > rank) : (j -= 1) {
                        nearest[j] = nearest[j - 1];
                    }
                    nearest[rank] = .{ .id = id_b, .dist = b_dist };
                    break;
                }
            }
        }
        expected[i] = nearest;
        bufs[i] = &neighbour_storage[i];
        points[i] = a_centre;
        excl_ids[i] = id_a;
    }
    const results = try tree.findNeighboursParallel(testing.io, &bufs, &points, &excl_ids, 3, 1);
    try testing.expectEqual(num_vols, results.len);
    for (&expected, results) |*nearest, neighbours| {
        try testing.expectEqualSlices(Tree.Neighbour, nearest, neighbours);
    }
    // single-point method must agree with the batch results
    var point_buf: [3]Tree.Neighbour = undefined;
    for (&expected, points, excl_ids) |*nearest, point, excl_id| {
        const found = try tree.findNeighboursSingle(&point_buf, point, excl_id, 3, 1);
        try testing.expectEqualSlices(Tree.Neighbour, nearest, found);
    }
}

test "occupancy counts are accurate" {
    const num_vols = 100;
    const seed = calc.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer calc.printErrorMessageForRandomSeed(seed);
    const Indexer = index.Indexer2f(2, 1, 2, .Morton);
    const Tree = SquareTree(Indexer, Box2f, u8);
    var tree = try Tree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, num_vols, 1);
    defer tree.deinit(test_alloc);
    var test_vols = try vol.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_vols,
        .{ .uniform = .{ .min = 0.005, .max = 0.04 } },
        .{ .uniform = .{ .min = 0.05, .max = 0.95 } },
    );
    defer test_vols.deinit(test_alloc);
    const boxes = test_vols.getRandomBodies(Box2f);
    const indexes = calc.getRange(Tree.ClientIdType, num_vols);
    try tree.addVolumes(boxes, &indexes);
    tree.updateBounds();
    // compute actual occupancy rates
    var expected_top_occupancy: [4]usize = [_]usize{0} ** 4;
    var expected_mle: usize = 0;
    for (0..Tree.num_leaves) |i| {
        const anc_index = Indexer.getLeafPredecessor(@truncate(i), 0);
        const num_vols_in_i = tree.getLeafVolumes(@truncate(i)).len;
        expected_top_occupancy[anc_index] += num_vols_in_i;
        expected_mle = @max(expected_mle, num_vols_in_i);
    }
    // compare with square tree methods
    var total_occupancy: usize = 0;
    for (0..4) |anc_index| {
        const anc_occupancy = tree.getOccupancyUnderNode(0, @truncate(anc_index));
        try testing.expectEqual(expected_top_occupancy[anc_index], anc_occupancy);
        total_occupancy += anc_occupancy;
    }
    try testing.expectEqual(num_vols, total_occupancy);
    const mle = tree.getMaxLeafOccupancy();
    try testing.expectEqual(expected_mle, @as(usize, @intCast(mle)));
}

test "draw square trees svg" {
    const IndexerM4x2 = index.Indexer2f(4, 1, 1, .Spring);
    const Trees = .{
        SquareTree(IndexerM4x2, Ball2f, u32),
        SquareTree(IndexerM4x2, Box2f, u32),
        SquareTree(IndexerM4x2, OrientedBox2f, u32),
    };
    const num_vols = 50;
    const seed = calc.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer calc.printErrorMessageForRandomSeed(seed);
    var test_vols = try vol.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_vols,
        .{ .uniform = .{ .min = 20, .max = 60 } },
        .{ .normal = .{ .mean = 512, .stddev = 250 } },
    );
    defer test_vols.deinit(test_alloc);
    const min_pt = Vec2f{ 0, 0 };
    const max_pt = Vec2f{ 1024, 1024 };
    // add the volumes to the tree and draw it
    inline for (Trees) |Tree| {
        var tree = try Tree.init(test_alloc, min_pt, max_pt, num_vols, 0);
        defer tree.deinit(test_alloc);
        const bodies = test_vols.getRandomBodies(Tree.VolumeType);
        const indexes = calc.getRange(u32, num_vols);
        try tree.addVolumes(bodies, &indexes);
        tree.updateBounds();
        var canvas = try tree.drawTreeSvg(test_alloc, true);
        defer canvas.deinit(test_alloc);
        var buf: [512]u8 = undefined;
        const path = try std.fmt.bufPrint(&buf, "{s}/{s}.html", .{ "test-out", @typeName(Tree) });
        try canvas.writeHtml(test_alloc, testing.io, path, true);
    }
}
