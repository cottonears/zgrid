const std = @import("std");
const calc = @import("maths/calc.zig");
const index = @import("maths/index.zig");
const rand = @import("maths/rand.zig");
const vol = @import("maths/volume.zig");
const para = @import("parallel.zig");
const draw = @import("draw.zig");
const math = std.math;
const Allocator = std.mem.Allocator;
const Io = std.Io;
const AtomicRangeIter = para.AtomicRangeIter;
const Vec2f = calc.Vec2f;
const Ball2f = vol.Ball2f;
const Box2f = vol.Box2f;
const OrientedBox2f = vol.OrientedBox2f;

/// A data structure that stores volumes + client IDs within an indexed region.
pub fn SquareTree(
    comptime IndexerType: type, // Indexer used to structure tree.
    comptime VolumeType: type, // Type of volumes stored in leaf nodes.
    comptime ClientIdType: type, // Caller-chosen ID type.
) type {
    return struct {
        indexer: Indexer,
        num_volumes: usize = 0, // number of volumes currently stored
        max_half_extent: Vec2f = @splat(0), // largest half-extent of any stored volume
        bounds_valid: bool = false, // false if bounds need to be updated
        node_bvs: [depth][]Box2f, // BVs for all nodes
        leaf_data: []Volume, // all volumes, sorted by leaf index
        leaf_ids: []ClientId, // client ids for leaf_data, in the same order
        leaf_starts: []StartIndex, // CSR offsets: leaf i owns leaf_data[starts[i]..starts[i + 1]]
        staged_data: []Volume, // volumes in insertion order, unsorted
        staged_ids: []ClientId, // client ids for staged_data, in the same order
        staged_indexes: []CurveIndex, // leaf indexes for staged volumes in the same order
        scratch_a: []CurveIndex, // scratch buffer A - for BFS search
        scratch_b: []CurveIndex, // scratch buffer B - for BFS search
        max_async_workers: u16,

        /// Errors exposed through public methods:
        pub const Error = error{
            InputLengthMismatch,
            BufferCapacityExceeded,
            LeafCapacityExceeded,
            TreeCapacityExceeded,
            TreeNotBuilt,
        };
        pub const ClientId = ClientIdType;
        pub const CurveIndex = IndexerType.CurveIndex;
        pub const Indexer = IndexerType;
        pub const Neighbour = struct { id: ClientId, dist: f32 };
        pub const Volume = VolumeType;
        pub const compressed = Indexer.top_levels > 1;
        pub const depth = Indexer.depth;
        pub const nodes_in_level = Indexer.nodes_in_level;
        pub const num_leaves = Indexer.num_leaves;
        const DataIndex = u16; // Used to index volumes within leaf nodes.
        const StartIndex = u24; // Offset into leaf_data/leaf_ids
        const VolIndex = struct { leaf: CurveIndex, offset: DataIndex }; // locates a stored volume
        const max_ring_size: usize = 4 * math.sqrt(num_leaves); // limits neighbourhood query growth
        // Tunable params below: current config is for a modern CPU (tested on Ryzen 9700X)
        const query_part_sizer: para.PartitionSizer = .{
            .min_tasks_per_part = 4,
            .min_parts_per_worker = 1,
            .max_parts_per_worker = 32,
        };
        const query_worker_buf_bytes = 16 * 1024; // stack-allocated bytes for each worker
        const query_worker_buf_pair_len = query_worker_buf_bytes / @sizeOf(ClientId);
        const update_bv_min_part_nodes = 64; // fewest nodes worth splitting across workers
        const update_bv_parts_per_worker = 4;
        const bv_top_lvl: u4 = blk: { // highest level where update computes bvs in parallel
            var lvl: u4 = depth - 1;
            while (lvl > 0 and nodes_in_level[lvl - 1] >= update_bv_min_part_nodes) lvl -= 1;
            break :blk lvl;
        };
        const bv_top_nodes = nodes_in_level[bv_top_lvl];
        const Self = @This();

        pub fn init(
            allocator: Allocator,
            bound_1: Vec2f, // a corner of the space to be covered
            bound_2: Vec2f, // the opposite corner of the space
            max_capacity: u32, // bounds the (heap-allocated) memory for storing volumes
            max_async_workers: u16, // limits the number of workers (0 defaults to cpu_count - 1)
        ) !Self {
            if (max_capacity > math.maxInt(StartIndex)) {
                return error.CapacityExceedsMaxStartIndex;
            }
            const indexer = try Indexer.init(bound_1, bound_2);
            const leaf_data = try allocator.alloc(Volume, max_capacity);
            errdefer allocator.free(leaf_data);
            const leaf_ids = try allocator.alloc(ClientId, max_capacity);
            errdefer allocator.free(leaf_ids);
            const leaf_starts = try allocator.alloc(StartIndex, num_leaves + 1);
            errdefer allocator.free(leaf_starts);
            @memset(leaf_starts, 0);
            const staged_data = try allocator.alloc(Volume, max_capacity);
            errdefer allocator.free(staged_data);
            const staged_ids = try allocator.alloc(ClientId, max_capacity);
            errdefer allocator.free(staged_ids);
            const staged_indexes = try allocator.alloc(CurveIndex, max_capacity);
            errdefer allocator.free(staged_indexes);
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
                .staged_data = staged_data,
                .staged_ids = staged_ids,
                .staged_indexes = staged_indexes,
                .scratch_a = scratch_buf_a,
                .scratch_b = scratch_buf_b,
                .max_async_workers = max_workers,
            };
        }

        pub fn deinit(self: *Self, allocator: Allocator) void {
            for (self.node_bvs) |v| allocator.free(v);
            allocator.free(self.scratch_a);
            allocator.free(self.scratch_b);
            allocator.free(self.staged_indexes);
            allocator.free(self.staged_ids);
            allocator.free(self.staged_data);
            allocator.free(self.leaf_starts);
            allocator.free(self.leaf_ids);
            allocator.free(self.leaf_data);
        }

        /// Adds volumes to the grid and stores their associated client ids (order must match).
        /// The volumes are staged: call `build` before querying.
        pub fn addVolumes(
            self: *Self,
            vols: []const Volume,
            client_ids: []const ClientId,
        ) Error!void {
            const n = self.num_volumes;
            if (n + vols.len > self.staged_data.len) return Error.TreeCapacityExceeded;
            if (vols.len != client_ids.len) return error.InputLengthMismatch;
            @memcpy(self.staged_data[n..][0..vols.len], vols);
            @memcpy(self.staged_ids[n..][0..client_ids.len], client_ids);
            self.num_volumes += vols.len;
            self.bounds_valid = false;
        }

        /// Sorts staged volumes into their final positions, then updates all nodes' BVs.
        /// Single-threaded but not thread-safe.
        pub fn build(self: *Self) !void {
            var idx_iter = AtomicRangeIter.init(0, self.num_volumes, 1);
            self.indexStagedVolumes(&idx_iter, &self.max_half_extent);
            try self.countSortStagedVolumes();
            var range_iter = AtomicRangeIter.init(0, nodes_in_level[0], 1);
            self.updateSubtreeBvsWorker(0, &range_iter);
            self.bounds_valid = true;
        }

        /// Sorts staged volumes into their final positions, then updates all nodes' BVs.
        /// Does work in parallel if the io implementation supports it; not thread-safe.
        pub fn buildParallel(self: *Self, io: Io) !void {
            const max_idx_workers = 8;
            var group: Io.Group = .init;
            errdefer group.cancel(io);
            // first index points and update max-half-extent (if compressed)
            const index_workers = @min(max_idx_workers, self.max_async_workers);
            var worker_mhes: [max_idx_workers]Vec2f = undefined;
            var count_iter = AtomicRangeIter.init(0, self.num_volumes, index_workers);
            for (0..index_workers) |i| {
                const args = .{ self, &count_iter, &worker_mhes[i] };
                group.async(io, indexStagedVolumes, args);
            }
            try group.await(io);
            var mhe: Vec2f = @splat(0);
            for (0..index_workers) |i| mhe = @max(mhe, worker_mhes[i]);
            self.max_half_extent = mhe;
            // then count-sort
            try self.countSortStagedVolumes();
            const target_parts = update_bv_parts_per_worker * @as(usize, self.max_async_workers);
            const parts = @min(bv_top_nodes, math.ceilPowerOfTwoAssert(usize, target_parts));
            const workers = @min(self.max_async_workers, parts);
            var range_iter = AtomicRangeIter.init(0, bv_top_nodes, parts);
            // build lower levels' bounding volumes updated in parallel
            for (0..workers) |_| {
                group.async(io, updateSubtreeBvsWorker, .{ self, bv_top_lvl, &range_iter });
            }
            try group.await(io);
            // build upper levels' bounding volumes serially
            var lvl = bv_top_lvl;
            while (lvl > 0) {
                lvl -= 1;
                self.updateLevelBvs(lvl, 0, nodes_in_level[lvl]);
            }
            self.bounds_valid = true;
        }

        // Indexes a range of staged volumes and computes their max half extent
        fn indexStagedVolumes(self: *Self, staging_iter: *AtomicRangeIter, max_half_ext: *Vec2f) void {
            var mhe: Vec2f = @splat(0);
            while (staging_iter.next()) |r| {
                for (r.start..r.end) |i| {
                    const v = self.staged_data[i];
                    const leaf_index = self.indexer.getLeafIndexForPoint(v.getCentre());
                    self.staged_indexes[i] = leaf_index;
                    if (compressed) {
                        const bb = v.getBoundingBox();
                        const he = calc.scaledVec(0.5, bb.max - bb.min);
                        mhe = @max(mhe, he);
                    }
                }
            }
            max_half_ext.* = mhe;
        }

        /// counts, sorts, and stores staged volumes into leaf_data in leaf_starts.
        fn countSortStagedVolumes(self: *Self) !void {
            const num_vols = self.num_volumes;
            if (comptime num_leaves < 65_536) { // small tree: store counts in temp stack array
                var leaf_counts: [num_leaves]DataIndex = undefined;
                @memset(&leaf_counts, 0);
                for (self.staged_indexes[0..num_vols]) |i| {
                    const data_index = leaf_counts[i];
                    if (data_index == math.maxInt(DataIndex)) return Error.LeafCapacityExceeded;
                    leaf_counts[i] = data_index + 1;
                }
                self.leaf_starts[0] = 0;
                var offset: StartIndex = 0;
                for (leaf_counts, 1..) |count, i| {
                    self.leaf_starts[i] = offset;
                    offset += count;
                }
            } else { // large tree: store counts in self.leaf_starts before offset pass
                @memset(self.leaf_starts, 0);
                for (self.staged_indexes[0..num_vols]) |i| {
                    const count = self.leaf_starts[i + 1];
                    if (count == math.maxInt(DataIndex)) return Error.LeafCapacityExceeded;
                    self.leaf_starts[i + 1] = count + 1;
                }
                self.leaf_starts[0] = 0;
                var offset: StartIndex = 0;
                for (0..self.leaf_starts.len) |i| {
                    const prev_count = self.leaf_starts[i];
                    self.leaf_starts[i] = offset;
                    offset += prev_count;
                }
            }
            // scatter staged vols + ids to their final position
            for (
                self.staged_data[0..num_vols],
                self.staged_ids[0..num_vols],
                self.staged_indexes[0..num_vols],
            ) |v, id, leaf_index| {
                const cursor = &self.leaf_starts[@as(usize, leaf_index) + 1];
                self.leaf_data[cursor.*] = v;
                self.leaf_ids[cursor.*] = id;
                cursor.* += 1;
            }
        }

        /// Computes leaf and ancestor BVs up until the top_lvl.
        fn updateSubtreeBvsWorker(self: *const Self, top_lvl: u4, range_iter: *AtomicRangeIter) void {
            const leaf_bvs = self.node_bvs[depth - 1];
            const num_leaf_succs = Indexer.getNumberLeafSuccessors(@truncate(top_lvl));
            // leaf BVs must be computed first
            while (range_iter.next()) |range| {
                const num_subtrees = range.end - range.start;
                for (range.start * num_leaf_succs..range.end * num_leaf_succs) |i| {
                    var box = vol.empty_box;
                    for (self.leaf_data[self.leaf_starts[i]..self.leaf_starts[i + 1]]) |v| {
                        box = vol.getBoundingBox(box, v);
                    }
                    leaf_bvs[i] = box;
                }
                var lvl: u4 = depth - 1;
                while (lvl > top_lvl) {
                    lvl -= 1;
                    const scale = nodes_in_level[lvl] / nodes_in_level[top_lvl];
                    self.updateLevelBvs(lvl, range.start * scale, num_subtrees * scale);
                }
            }
        }

        /// Fits BVs around children for count nodes on lvl (starting at first).
        fn updateLevelBvs(self: *const Self, lvl: u4, first: usize, count: usize) void {
            const child_bvs = self.node_bvs[lvl + 1];
            for (self.node_bvs[lvl][first..][0..count], first..) |*bv, j| {
                var box = vol.empty_box;
                const first_child = Indexer.getFirstChild(@truncate(j));
                for (child_bvs[first_child..][0..Indexer.num_children]) |c| {
                    box = vol.getBoundingBox(box, c);
                }
                bv.* = box;
            }
        }

        /// Removes all volumes stored in leaf-nodes of the grid.
        pub fn clear(self: *Self) void {
            self.bounds_valid = false;
            self.max_half_extent = @splat(0);
            self.num_volumes = 0;
        }

        /// Diagnostic: the largest number of volumes staged in any single leaf.
        pub fn getMaxLeafOccupancy(self: *const Self) !StartIndex {
            if (!self.bounds_valid) return Error.TreeNotBuilt;
            var max_count: StartIndex = 0;
            for (1..self.leaf_starts.len) |i| {
                const count = self.leaf_starts[i] - self.leaf_starts[i - 1];
                max_count = @max(max_count, count);
            }
            return max_count;
        }

        /// Gets the total number of volumes stored under a node in this tree's hierarchy.
        /// Counts the volumes stored under all successors' leaves.
        pub fn getOccupancyUnderNode(self: *const Self, lvl: u4, node: CurveIndex) !usize {
            if (!self.bounds_valid) return Error.TreeNotBuilt;
            std.debug.assert(lvl < depth);
            const succ_start = Indexer.getFirstLeafSuccessor(@truncate(lvl), node);
            const succ_end: usize = succ_start + Indexer.getNumberLeafSuccessors(@truncate(lvl));
            return self.leaf_starts[succ_end] - self.leaf_starts[succ_start];
        }

        /// Relocates the tree to a new position.
        /// Tree must be empty: call `clear` first.
        pub fn relocate(self: *Self, new_min: Vec2f, new_max: Vec2f) !void {
            if (self.num_volumes > 0) return error.CannotRelocateOccupiedTree;
            self.indexer = try Indexer.init(new_min, new_max);
        }

        /// Returns id pairs for stored volumes that overlap with the provided query volumes.
        /// Requires `build` to have been called since the last `addVolume`.
        /// Single-threaded (no io dependency) but not thread-safe (writes to scratch bufs).
        pub fn findExtOverlaps(
            self: *Self,
            overlap_buf: [][2]ClientId,
            query_ids: []const ClientId,
            query_vols: anytype,
        ) Error![][2]ClientId {
            if (query_ids.len != query_vols.len) return error.InputLengthMismatch;
            if (!self.bounds_valid) return error.TreeNotBuilt;
            var range_iter = AtomicRangeIter.init(0, query_ids.len, 1);
            var shared_buf = para.SharedBuffer([2]ClientId).init(overlap_buf);
            self.findExtOverlapsWorker(
                self.scratch_a[0..num_leaves],
                self.scratch_b[0..num_leaves],
                &range_iter,
                &shared_buf,
                query_ids,
                query_vols,
            );
            return shared_buf.getItems();
        }

        /// Returns id pairs for stored volumes that overlap with the provided query volumes.
        /// Requires `build` to have been called since the last `addVolume`.
        /// Does work in parallel if the io implementation supports it; not thread-safe.
        pub fn findExtOverlapsParallel(
            self: *Self,
            io: Io,
            overlap_buf: [][2]ClientId,
            query_ids: []const ClientId,
            query_vols: anytype,
        ) ![][2]ClientId {
            if (query_ids.len != query_vols.len) return error.InputLengthMismatch;
            if (!self.bounds_valid) return error.TreeNotBuilt;
            if (query_ids.len == 0) return overlap_buf[0..0];
            // NOTE: async rejects anytype and comptime params, so the worker needs a concrete wrapper.
            const Worker = struct {
                fn run(
                    tree: *const Self,
                    scratch_a: []CurveIndex,
                    scratch_b: []CurveIndex,
                    range_iter: *AtomicRangeIter,
                    shared_buf: *para.SharedBuffer([2]ClientId),
                    ids: []const ClientId,
                    vols: @TypeOf(query_vols),
                ) void {
                    tree.findExtOverlapsWorker(scratch_a, scratch_b, range_iter, shared_buf, ids, vols);
                }
            };
            const num_parts = query_part_sizer.getParts(query_ids.len, self.max_async_workers);
            const num_workers = query_part_sizer.getWorkers(num_parts, self.max_async_workers);
            var range_iter = AtomicRangeIter.init(0, query_ids.len, num_parts);
            var shared_buf = para.SharedBuffer([2]ClientId).init(overlap_buf);
            var group: Io.Group = .init;
            errdefer group.cancel(io);
            for (0..num_workers) |i| {
                group.async(io, Worker.run, .{
                    self,
                    self.scratch_a[i * num_leaves ..][0..num_leaves],
                    self.scratch_b[i * num_leaves ..][0..num_leaves],
                    &range_iter,
                    &shared_buf,
                    query_ids,
                    query_vols,
                });
            }
            try group.await(io);
            return shared_buf.getItems();
        }

        // TODO: refactor this so it doesn't take / return an id!
        /// Returns id pairs for stored volumes that overlap with the provided query volume.
        /// Requires `build` to have been called since the last `addVolume`.
        /// Single-threaded (no io dependency) but not thread-safe (writes to scratch bufs).
        pub fn findExtOverlapsSingle(
            self: *Self,
            overlap_buf: [][2]ClientId,
            query_id: ClientId,
            query_vol: anytype,
        ) Error![][2]ClientId {
            const query_ids = [_]ClientId{query_id};
            const query_vols = [_]@TypeOf(query_vol){query_vol};
            return self.findExtOverlaps(overlap_buf, &query_ids, &query_vols);
        }

        fn findExtOverlapsWorker(
            self: *const Self,
            scratch_a: []CurveIndex,
            scratch_b: []CurveIndex,
            range_iter: *AtomicRangeIter,
            shared_buf: *para.SharedBuffer([2]ClientId),
            query_ids: []const ClientId,
            query_vols: anytype,
        ) void {
            var pair_buf: [query_worker_buf_pair_len][2]ClientId = undefined;
            var work_list = std.ArrayList([2]ClientId).initBuffer(&pair_buf);
            while (range_iter.next()) |range| {
                for (query_ids[range.start..range.end], query_vols[range.start..range.end]) |id, v| {
                    self.findOverlapsBfs(shared_buf, &work_list, scratch_a, scratch_b, id, v, 0, 0);
                }
            }
            shared_buf.appendSlice(work_list.items); // publish whatever is left staged
        }

        /// Returns ids for every pair of stored volumes that overlap with each other.
        /// Requires `build` to have been called since the last `addVolume`.
        /// Single-threaded (no io dependency) but not thread-safe (writes to scratch bufs).
        pub fn findSelfOverlaps(self: *Self, overlap_buf: [][2]ClientId) Error![][2]ClientId {
            if (!self.bounds_valid) return error.TreeNotBuilt;
            var range_iter = AtomicRangeIter.init(0, self.num_volumes, 1);
            var shared_buf = para.SharedBuffer([2]ClientId).init(overlap_buf);
            self.findSelfOverlapWorker(
                self.scratch_a[0..num_leaves],
                self.scratch_b[0..num_leaves],
                &range_iter,
                &shared_buf,
            );
            return shared_buf.getItems();
        }

        /// Returns ids for every pair of stored volumes that overlap with each other.
        /// Requires `build` to have been called since the last `addVolume`.
        /// Does work in parallel if the io implementation supports it; not thread-safe.
        pub fn findSelfOverlapsParallel(
            self: *Self,
            io: Io,
            overlap_buf: [][2]ClientId,
        ) ![][2]ClientId {
            if (!self.bounds_valid) return error.TreeNotBuilt;
            const num_volumes = self.num_volumes;
            if (num_volumes == 0) return overlap_buf[0..0];
            const num_parts = query_part_sizer.getParts(num_volumes, self.max_async_workers);
            const num_workers = query_part_sizer.getWorkers(num_parts, self.max_async_workers);
            var range_iter = AtomicRangeIter.init(0, num_volumes, num_parts);
            var shared_buf = para.SharedBuffer([2]ClientId).init(overlap_buf);
            var group: Io.Group = .init;
            errdefer group.cancel(io);
            for (0..num_workers) |i| {
                group.async(io, findSelfOverlapWorker, .{
                    self,
                    self.scratch_a[i * num_leaves ..][0..num_leaves],
                    self.scratch_b[i * num_leaves ..][0..num_leaves],
                    &range_iter,
                    &shared_buf,
                });
            }
            try group.await(io);
            return shared_buf.getItems();
        }

        fn findSelfOverlapWorker(
            self: *const Self,
            scratch_a: []CurveIndex,
            scratch_b: []CurveIndex,
            range_iter: *AtomicRangeIter,
            shared_buf: *para.SharedBuffer([2]ClientId),
        ) void {
            // results copied to a small buffer on the stack and flushed to the shared buffer as needed
            var work_buf: [query_worker_buf_pair_len][2]ClientId = undefined;
            var work_list = std.ArrayList([2]ClientId).initBuffer(&work_buf);
            while (range_iter.next()) |range| {
                if (range.start >= range.end) continue;
                var leaf_cursor = self.flatIndexToLeafIndex(range.start);
                for (range.start..range.end) |i| {
                    const vol_index = self.nextVolIndex(&leaf_cursor, i);
                    self.findOverlapsBfs(
                        shared_buf,
                        &work_list,
                        scratch_a,
                        scratch_b,
                        self.leaf_ids[i],
                        self.leaf_data[i],
                        vol_index.leaf,
                        vol_index.offset + 1,
                    );
                }
            }
            shared_buf.appendSlice(work_list.items);
        }

        /// Performs a BFS for stored volumes that overlap with the provided query volume.
        fn findOverlapsBfs(
            self: *const Self,
            shared_buf: *para.SharedBuffer([2]ClientId),
            work_list: *std.ArrayList([2]ClientId),
            slice_a: []CurveIndex,
            slice_b: []CurveIndex,
            query_id: ClientId,
            query_vol: anytype,
            start_leaf: CurveIndex,
            start_vol_index: DataIndex,
        ) void {
            // search through higher-level nodes first
            const query_aabb: Box2f = query_vol.getBoundingBox();
            var search_list = std.ArrayList(CurveIndex).initBuffer(slice_a);
            if (compressed) { // check the neighbouring level 0 nodes only
                const n_box: Box2f = .{
                    .min = query_aabb.min - self.max_half_extent,
                    .max = query_aabb.max + self.max_half_extent,
                };
                self.indexer.getTopLevelIndexesForBox(&search_list, n_box, start_leaf);
            } else { // check all level 0 nodes
                const pred_0 = Indexer.getLeafPredecessor(start_leaf, 0);
                for (pred_0..nodes_in_level[0]) |k| search_list.appendAssumeCapacity(@intCast(k));
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
                    for (start..end) |k| next_list.appendAssumeCapacity(@intCast(k));
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
                    if (work_list.items.len == query_worker_buf_pair_len) { // publish a full batch
                        shared_buf.appendSlice(work_list.items);
                        work_list.clearRetainingCapacity();
                    }
                    work_list.appendAssumeCapacity(.{ query_id, id });
                }
            }
        }

        /// Finds stored volumes nearest to each query point, nearest-first.
        /// Search stops when k volumes are found, or there are no more candidates within `max_dist`.
        /// Requires `build` to have been called since the last `addVolume`.
        pub fn findNeighbours(
            self: *const Self,
            bufs: [][]Neighbour,
            points: []const Vec2f,
            excl_ids: []const ?ClientId,
            k: u16,
            max_dist: f32,
        ) Error![][]Neighbour {
            if (bufs.len != points.len or bufs.len != excl_ids.len) return error.InputLengthMismatch;
            if (!self.bounds_valid) return error.TreeNotBuilt;
            for (bufs) |buf| if (k > buf.len) return error.BufferCapacityExceeded;
            var range_iter = AtomicRangeIter.init(0, bufs.len, 1);
            self.findNeighboursWorker(bufs, points, excl_ids, k, max_dist, &range_iter);
            return bufs;
        }

        /// Finds stored volumes nearest to the query point, nearest-first.
        /// Search stops when k volumes are found, or there are no more candidates within `max_dist`.
        /// Requires `build` to have been called since the last `addVolume`.
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
            if (!self.bounds_valid) return error.TreeNotBuilt;
            for (bufs) |buf| if (k > buf.len) return error.BufferCapacityExceeded;
            if (bufs.len == 0) return bufs;
            const num_parts = query_part_sizer.getParts(bufs.len, self.max_async_workers);
            const num_workers = query_part_sizer.getWorkers(num_parts, self.max_async_workers);
            var range_iter = AtomicRangeIter.init(0, bufs.len, num_parts);
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
        /// Requires `build` to have been called since the last `addVolume`.
        pub fn findNeighboursSingle(
            self: *const Self,
            buf: []Neighbour,
            point: Vec2f,
            excl_id: ?ClientId,
            k: u16,
            max_dist: f32,
        ) Error![]Neighbour {
            if (!self.bounds_valid) return error.TreeNotBuilt;
            return self.neighboursForPoint(buf, point, excl_id, k, max_dist);
        }

        fn findNeighboursWorker(
            self: *const Self,
            bufs: [][]Neighbour,
            points: []const Vec2f,
            excl_ids: []const ?ClientId,
            k: u16,
            max_dist: f32,
            range_iter: *AtomicRangeIter,
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

        /// Gets the client ids stored in the specified leaf node in insertion order.
        fn getLeafIds(self: *const Self, leaf_num: CurveIndex) []const ClientId {
            const start = self.leaf_starts[leaf_num];
            const end = self.leaf_starts[@as(usize, leaf_num) + 1];
            return self.leaf_ids[start..end];
        }

        /// Gets the volumes stored in the specified leaf node in insertion order.
        fn getLeafVolumes(self: *const Self, leaf_num: CurveIndex) []const Volume {
            const start = self.leaf_starts[leaf_num];
            const end = self.leaf_starts[@as(usize, leaf_num) + 1];
            return self.leaf_data[start..end];
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
            return self.leaf_starts[@as(usize, leaf_index) + 1];
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
    };
}

const testing = std.testing;
const test_alloc = testing.allocator;
const test_capacity = 1000;

test "square tree init + deinit" {
    // check for memory leaks
    const Tree2x2 = SquareTree(index.Indexer2f(.Morton4, 1), Ball2f, u16);
    var qt = try Tree2x2.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 0);
    defer qt.deinit(test_alloc);
    const Tree4x2 = SquareTree(index.Indexer2f(.Zigzag16, 1), Ball2f, u16);
    var ht = try Tree4x2.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 0);
    defer ht.deinit(test_alloc);
}

test "square tree add remove" {
    const IndexerM2x4 = index.Indexer2f(.Morton16, 1);
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
    try qt.build();

    // check volumes retrieved by id come back unchanged
    for (0..QuadTree.num_leaves) |leaf_num_usize| {
        const leaf_num: QuadTree.CurveIndex = @intCast(leaf_num_usize);
        for (qt.getLeafVolumes(leaf_num), qt.getLeafIds(leaf_num)) |v, id| {
            try testing.expectEqual(test_bodies[id].centre, v.centre);
            try testing.expectEqual(test_bodies[id].radius, v.radius);
        }
    }
    qt.clear();
    try testing.expectEqual(0, qt.num_volumes);
}

test "hex tree overlap ball" {
    const HexTree2 = SquareTree(index.Indexer2f(.Zigzag16, 1), Ball2f, u16);
    var tree = try HexTree2.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 8);
    defer tree.deinit(test_alloc);
    var balls = [3]Ball2f{
        .{ .centre = .{ 0.2, 0.0 }, .radius = 0.4 },
        .{ .centre = .{ 0.2, 0.5 }, .radius = 0.2 },
        .{ .centre = .{ 0.2, 0.7 }, .radius = 0.1 },
    };
    const indexes = calc.getRange(u16, balls.len);
    try tree.addVolumes(balls[0..], &indexes);
    try tree.buildParallel(testing.io);
    var pairs_buff: [16][2]u16 = undefined;
    const query_ids = [_]u16{ 4, 5, 6 };
    const query_regions = [_]Ball2f{
        .{ .centre = .{ 0.9, 0.5 }, .radius = 0.1 },
        .{ .centre = .{ -3.5, -3.5 }, .radius = 1.0 },
        .{ .centre = .{ 0.2, 0.5 }, .radius = 0.2 },
    };
    // query balls 4 and 5 overlap nothing; 6 overlaps all 3 stored balls
    const ext_overlaps = try tree.findExtOverlapsParallel(
        testing.io,
        &pairs_buff,
        &query_ids,
        &query_regions,
    );
    const expected_ext = [_][2]u16{ .{ 0, 6 }, .{ 1, 6 }, .{ 2, 6 } };
    calc.sortPairsLexicographic(u16, ext_overlaps);
    try testing.expectEqualSlices([2]u16, &expected_ext, ext_overlaps);
    // a overlaps b, and b overlaps c, but a does not overlap c.
    const self_overlaps = try tree.findSelfOverlapsParallel(testing.io, &pairs_buff);
    calc.sortPairsLexicographic(u16, self_overlaps);
    const expected_self = [_][2]u16{ .{ 0, 1 }, .{ 1, 2 } };
    try testing.expectEqualSlices([2]u16, &expected_self, self_overlaps);
}

test "short overlap buffer returns a capacity error" {
    const Tree = SquareTree(index.Indexer2f(.Zigzag16, 1), Ball2f, u32);
    var tree = try Tree.init(test_alloc, .{ -1, -1 }, .{ 1, 1 }, 16, 0);
    defer tree.deinit(test_alloc);
    const balls = [_]Ball2f{.{ .centre = .{ 0, 0 }, .radius = 0.5 }} ** 16;
    const ids = calc.getRange(u32, balls.len);

    // add the volumes and check the expected error ius retturned
    try tree.addVolumes(&balls, &ids);
    try tree.build();
    var buf: [32][2]u32 = undefined;
    const result_st = tree.findSelfOverlaps(buf[0..4]);
    const result_mt = tree.findSelfOverlapsParallel(testing.io, buf[0..4]);
    try testing.expectError(error.BufferCapacityExceeded, result_st);
    try testing.expectError(error.BufferCapacityExceeded, result_mt);
}

test "hex tree overlap box" {
    const HexTree2 = SquareTree(index.Indexer2f(.Zigzag16, 1), Box2f, u16);
    var tree = try HexTree2.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, test_capacity, 0);
    defer tree.deinit(test_alloc);
    const boxes = [_]Box2f{
        .{ .min = .{ -0.2, -0.4 }, .max = .{ 0.6, 0.4 } },
        .{ .min = .{ 0.0, 0.3 }, .max = .{ 0.4, 0.7 } },
        .{ .min = .{ 0.1, 0.6 }, .max = .{ 0.3, 0.8 } },
    };

    const indexes = calc.getRange(u16, boxes.len);
    try tree.addVolumes(&boxes, &indexes);
    try tree.buildParallel(testing.io);
    const query_ids = [_]u16{ 4, 5, 6 };
    const query_regions = [_]Box2f{
        .{ .min = .{ 0.8, 0.4 }, .max = .{ 1.0, 0.6 } },
        .{ .min = @splat(-4.5), .max = @splat(-2.5) },
        .{ .min = .{ 0.0, 0.3 }, .max = .{ 0.4, 0.7 } },
    };
    // query boxes 4 and 5 overlap nothing; 6 overlaps all 3 stored boxes
    var id_buff: [16][2]u16 = undefined;
    const ext_overlaps = try tree.findExtOverlaps(&id_buff, &query_ids, &query_regions);
    const expected_ext = [_][2]u16{ .{ 0, 6 }, .{ 1, 6 }, .{ 2, 6 } };
    calc.sortPairsLexicographic(u16, ext_overlaps);
    try testing.expectEqualSlices([2]u16, &expected_ext, ext_overlaps);
    // a overlaps b, and b overlaps c, but a does not overlap c.
    const self_overlaps = try tree.findSelfOverlapsParallel(testing.io, &id_buff);
    calc.sortPairsLexicographic(u16, self_overlaps);
    const expected_self = [_][2]u16{ .{ 0, 1 }, .{ 1, 2 } };
    try testing.expectEqualSlices([2]u16, &expected_self, self_overlaps);
}

// TODO: add simple test for external line vs tree volumes

test "tree occupancy counts are accurate" {
    const seed = rand.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer rand.printErrorMessageForRandomSeed(seed);
    var pos_dist = rand.ProbDensityFunc{
        .normal = .{ .mean = 0.0, .stddev = 1.0 },
    };
    const Indexer = index.Indexer2f(.Zigzag16, 1);
    const Tree = SquareTree(Indexer, Ball2f, u16);
    var tree = try Tree.init(test_alloc, .{ -4, -4 }, .{ 4, 4 }, test_capacity, 1);
    defer tree.deinit(test_alloc);
    var centres: [test_capacity]Vec2f = undefined;
    var balls: [test_capacity]Ball2f = undefined;
    pos_dist.fillVec2f(prng.random(), centres[0..]);
    for (0..test_capacity) |i| balls[i] = .{ .centre = centres[i], .radius = 0.1 };
    const indexes = calc.getRange(Tree.ClientId, test_capacity);
    try tree.addVolumes(&balls, &indexes);
    try tree.build();

    // compute leaf occupancy rates
    var leaf_counts = [_]usize{0} ** Tree.num_leaves;
    for (centres) |centre| {
        const leaf = tree.indexer.getLeafIndexForPoint(centre);
        leaf_counts[leaf] += 1;
    }
    var max_leaf_count: usize = 0;
    for (leaf_counts) |c| {
        max_leaf_count = @max(max_leaf_count, c);
    }
    const tree_max_leaf_occ = try tree.getMaxLeafOccupancy();
    try testing.expectEqual(max_leaf_count, tree_max_leaf_occ);

    // compute top-occupancy
    var sum_top_occupancy: usize = 0;
    for (0..Tree.nodes_in_level[0]) |i| {
        const leaf_start = Indexer.getFirstLeafSuccessor(0, @truncate(i));
        const leaf_end = leaf_start + Indexer.getNumberLeafSuccessors(0);
        var occupancy_i: usize = 0;
        for (leaf_start..leaf_end) |j| occupancy_i += leaf_counts[j];
        const tree_i_occupancy = try tree.getOccupancyUnderNode(0, @truncate(i));
        try testing.expectEqual(occupancy_i, tree_i_occupancy);
        sum_top_occupancy += occupancy_i;
    }
    try testing.expectEqual(test_capacity, sum_top_occupancy);
}
