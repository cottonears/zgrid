const std = @import("std");
const builtin = @import("builtin");
const zgrid = @import("zgrid");
const calc = zgrid.calc;
const index = zgrid.index;
const st = zgrid.square_tree;
const vol = zgrid.volume;
const ArgsIter = std.process.Args.Iterator;
const Vec2f = calc.Vec2f;
const Ball2f = vol.Ball2f;
const Box2f = vol.Box2f;
const Line2f = vol.Line2f;
const OrientedBox2f = vol.OrientedBox2f;
const timer = std.Io.Clock.awake;
const max_capacity = 200_000;
const min_trials = 10;
const min_num_vols = 100;
const usage_msg =
    \\Usage: zgrid-bench [options]
    \\  -i: Set an input file (csv) to load test volumes from (see readme for correct format)
    \\  -p: String defining pdf used to generate test volume positions; default "N(5,2.5)"
    \\      Parameters in parentheses for (N)ormal: (mean,std_dev).
    \\      Parameters in parentheses for (U)niform: (min,max).
    \\  -s: String for pdf used to generate test volumes sizes; default "U(0.001,0.05)"
    \\  -t: Number of times to repeat each benchmark (for stable timing averages); default = 30
    \\  -v: Number of volumes to generate (ignored if using -i); default = 20000 (2000 in debug)
    \\
;
var input_file: ?[]const u8 = null;
var output_dir: ?[]const u8 = null;
var random_vols: vol.TestVolumes = undefined;
var position_dist: calc.ProbDensityFunc = .{ .normal = .{ .mean = 5.0, .stddev = 1.5 } };
var size_dist: calc.ProbDensityFunc = .{ .uniform = .{ .min = 0.001, .max = 0.05 } };
var num_vols: u24 = if (builtin.mode == .ReleaseFast) 20_000 else 2_000;
var num_trials: u8 = 30;
const untimed_trials = 3;

/// Fetches the value following a flag, erroring with the usage message if none was supplied.
fn nextArgValue(args_iter: *ArgsIter, flag: []const u8) ![:0]const u8 {
    return args_iter.next() orelse {
        std.debug.print("Missing value for '{s}'.\n{s}", .{ flag, usage_msg });
        return error.MissingArgumentValue;
    };
}

fn processArgs(io: std.Io, args_iter: *ArgsIter) !void {
    while (args_iter.next()) |arg| {
        if (std.mem.eql(u8, arg, "-i")) {
            const file = try nextArgValue(args_iter, arg);
            const cwd = std.Io.Dir.cwd();
            cwd.access(io, file, .{ .read = true }) catch |err| {
                std.debug.print("Unable to read from {s}\n", .{file});
                return err;
            };
            input_file = file;
        } else if (std.mem.eql(u8, arg, "-p")) {
            const pdf_str = try nextArgValue(args_iter, arg);
            position_dist = calc.ProbDensityFunc.fromPdfString(pdf_str) catch {
                std.debug.print("Could not parse pdf string '{s}':\n{s}", .{ pdf_str, usage_msg });
                return error.InvalidPdfArgument;
            };
        } else if (std.mem.eql(u8, arg, "-s")) {
            const pdf_str = try nextArgValue(args_iter, arg);
            size_dist = calc.ProbDensityFunc.fromPdfString(pdf_str) catch {
                std.debug.print("Could not parse pdf string '{s}':\n{s}", .{ pdf_str, usage_msg });
                return error.InvalidPdfArgument;
            };
        } else if (std.mem.eql(u8, arg, "-t")) {
            const val_str = try nextArgValue(args_iter, arg);
            num_trials = std.fmt.parseInt(u8, val_str, 10) catch {
                std.debug.print("Could not parse trial count '{s}':\n{s}", .{ val_str, usage_msg });
                return error.InvalidTrialCount;
            };
            if (num_trials < min_trials) {
                std.debug.print(
                    "Requested {} trials is below the minimum required ({}).\n{s}",
                    .{ num_trials, min_trials, usage_msg },
                );
                return error.TooFewTrials;
            }
        } else if (std.mem.eql(u8, arg, "-v")) {
            const val_str = try nextArgValue(args_iter, arg);
            num_vols = std.fmt.parseInt(u24, val_str, 10) catch {
                std.debug.print("Could not parse volume count '{s}':\n{s}", .{ val_str, usage_msg });
                return error.InvalidVolumeCount;
            };
            if (min_num_vols > num_vols or num_vols > max_capacity) {
                std.debug.print(
                    "Requested {} volumes outside supported range ({} - {}).\n{s}",
                    .{ num_vols, min_num_vols, max_capacity, usage_msg },
                );
                return error.InvalidNumberVolumes;
            }
        } else {
            std.debug.print("Unrecognised argument '{s}'.\n{s}", .{ arg, usage_msg });
            return error.UnrecognisedArgument;
        }
    }
}

pub fn main(init: std.process.Init) !void {
    const allocator = init.arena.allocator();
    var args_iter = try ArgsIter.initAllocator(init.minimal.args, allocator);
    defer args_iter.deinit();
    _ = args_iter.next(); // skip the program name
    try processArgs(init.io, &args_iter);

    if (input_file) |file| {
        random_vols = try vol.TestVolumes.initCsv(allocator, init.io, file);
        std.debug.print("Running benchmarks using volumes loaded from '{s}'...\n", .{file});
    } else {
        var prng = std.Random.DefaultPrng.init(0);
        random_vols = try vol.TestVolumes.initRandom(allocator, prng.random(), num_vols, size_dist, position_dist);
        std.debug.print("Running benchmarks for {} vols...\n", .{num_vols});
    }
    defer random_vols.deinit(allocator);

    try benchmarkIndexing(allocator, init.io);
    try benchmarkOverlapChecks(allocator, init.io);
    try benchmarkSquareTrees(allocator, init.io);
}

fn elapsedNs(t1: std.Io.Timestamp, t2: std.Io.Timestamp) f64 {
    return @floatFromInt(std.Io.Timestamp.durationTo(t1, t2).toNanoseconds());
}

fn benchmarkIndexing(allocator: std.mem.Allocator, io: std.Io) !void {
    const IndexerTypes = [_]type{
        index.Indexer2f(2, 1, 2, index.Curve.Morton),
        index.Indexer2f(2, 1, 3, index.Curve.Morton),
        index.Indexer2f(2, 1, 4, index.Curve.Morton),
        index.Indexer2f(2, 1, 5, index.Curve.Morton),
        index.Indexer2f(2, 1, 6, index.Curve.Morton),
        index.Indexer2f(2, 1, 7, index.Curve.Morton),
        index.Indexer2f(4, 1, 1, index.Curve.Morton),
        index.Indexer2f(4, 1, 2, index.Curve.Spring),
        index.Indexer2f(4, 1, 3, index.Curve.Zigzag),
        index.Indexer2f(4, 1, 4, index.Curve.Zigzag),
    };
    const headers: [1][]const u8 = .{" time (ns/pt) "};
    const formats: [1][]const u8 = .{" {d:>12.3} "};
    const num_total_rows = IndexerTypes.len * num_trials;
    var table = try DataTable(f64, 1, headers, formats).init(allocator, num_total_rows);
    defer table.deinit(allocator);

    inline for (IndexerTypes) |Indexer| {
        table.clear();
        const pt1 = random_vols.balls.items[0].centre;
        const pt2 = random_vols.balls.items[1].centre;
        var indexer = try Indexer.init(pt1, pt2);
        var indexes = try allocator.alloc(Indexer.CurveIndex, random_vols.boxes.items.len);
        defer allocator.free(indexes);

        // measure inter-leaf distances (crude indicator of how well the curve preseres locality)
        var index_dist_sum: f64 = 0.0;
        var next_centre = indexer.getLeafCellBoundary(0).getCentre();
        for (1..Indexer.num_leaves) |i| {
            const centre_i = indexer.getLeafCellBoundary(@truncate(i)).getCentre();
            index_dist_sum += calc.norm(centre_i - next_centre);
            next_centre = centre_i;
        }
        const avg_il_dist = index_dist_sum / @as(f64, @floatFromInt(Indexer.num_leaves - 1));

        // untimed warmup trials
        for (0..untimed_trials) |_| {
            for (random_vols.boxes.items, 0..) |b, i| {
                const index_of_b = indexer.getLeafIndexForPoint(b.getCentre());
                indexes[i] = index_of_b;
            }
        }

        // timed trails
        for (0..num_trials) |_| {
            const t_0 = timer.now(io);
            for (random_vols.boxes.items, 0..) |b, i| {
                const index_of_b = indexer.getLeafIndexForPoint(b.getCentre());
                indexes[i] = index_of_b;
            }
            const t_1 = timer.now(io);
            const avg_t = elapsedNs(t_0, t_1) / @as(f64, @floatFromInt(random_vols.boxes.items.len));
            try table.addRow(.{avg_t});
        }

        const stats_str = try table.getStatsTable(allocator);
        defer allocator.free(stats_str);
        std.debug.print(
            "Indexing benchmark for {s}: inter-leaf dist {d:.4}:\n{s}\n",
            .{ Indexer.type_label, avg_il_dist, stats_str },
        );
    }
}

fn benchmarkOverlapChecks(allocator: std.mem.Allocator, io: std.Io) !void {
    const num_cols = 5;
    const headers: [num_cols][]const u8 = .{
        " ball-ball ", " box-box ", " ball-box ", " obb-box ", " line-box ",
    };
    const formats: [num_cols][]const u8 = .{
        " {d:>9.3} ", " {d:>7.3} ", " {d:>8.3} ", " {d:>7.3} ", " {d:>8.3} ",
    };
    var table = try DataTable(f64, num_cols, headers, formats).init(allocator, num_trials);
    defer table.deinit(allocator);
    var first_ball = random_vols.balls.items[0];
    first_ball.radius = first_ball.radius * 25;
    const first_box = random_vols.boxes.items[0];
    const query_box = first_box.getScaled(25);
    const query_obb = random_vols.oriented_boxes.items[0].getScaled(25);
    const query_line = Line2f{ .start = first_box.min, .end = first_box.max };

    // untimed warmup trials
    var n: usize = 0;
    for (0..untimed_trials) |_| {
        for (random_vols.balls.items) |b| n += if (vol.checkVolumesOverlap(first_ball, b)) 1 else 0;
        for (random_vols.boxes.items) |b| n += if (vol.checkVolumesOverlap(query_box, b)) 1 else 0;
        for (random_vols.balls.items) |b| n += if (vol.checkVolumesOverlap(query_box, b)) 1 else 0;
        for (random_vols.boxes.items) |b| n += if (vol.checkVolumesOverlap(first_ball, b)) 1 else 0;
        for (random_vols.boxes.items) |b| n += if (vol.checkVolumesOverlap(query_obb, b)) 1 else 0;
        for (random_vols.boxes.items) |b| n += if (vol.checkVolumesOverlap(query_line, b)) 1 else 0;
    }

    // timed trials
    var overlap_count: u32 = 0;
    const ball_checks: f64 = @floatFromInt(random_vols.balls.items.len);
    const box_checks: f64 = @floatFromInt(random_vols.boxes.items.len);
    const mixed_checks: f64 = @floatFromInt(random_vols.balls.items.len + random_vols.boxes.items.len);
    for (0..num_trials) |_| {
        const t_0 = timer.now(io);
        for (random_vols.balls.items) |b| {
            overlap_count += if (vol.checkVolumesOverlap(first_ball, b)) 1 else 0;
        }
        const t_1 = timer.now(io);
        for (random_vols.boxes.items) |b| {
            overlap_count += if (vol.checkVolumesOverlap(query_box, b)) 1 else 0;
        }
        const t_2 = timer.now(io);
        for (random_vols.balls.items) |b| {
            overlap_count += if (vol.checkVolumesOverlap(query_box, b)) 1 else 0;
        }
        for (random_vols.boxes.items) |b| {
            overlap_count += if (vol.checkVolumesOverlap(first_ball, b)) 1 else 0;
        }
        const t_3 = timer.now(io);
        for (random_vols.boxes.items) |b| {
            overlap_count += if (vol.checkVolumesOverlap(query_obb, b)) 1 else 0;
        }
        const t_4 = timer.now(io);
        for (random_vols.boxes.items) |b| {
            overlap_count += if (vol.checkVolumesOverlap(query_line, b)) 1 else 0;
        }
        const t_5 = timer.now(io);
        try table.addRow(.{
            elapsedNs(t_0, t_1) / ball_checks,
            elapsedNs(t_1, t_2) / box_checks,
            elapsedNs(t_2, t_3) / mixed_checks,
            elapsedNs(t_3, t_4) / box_checks,
            elapsedNs(t_4, t_5) / box_checks,
        });
    }

    std.debug.print("Overlap checks: found {} overlaps:\n", .{overlap_count});
    const stats_str = try table.getStatsTable(allocator);
    defer allocator.free(stats_str);
    std.debug.print("{s}\n", .{stats_str});
}

fn benchmarkSquareTrees(allocator: std.mem.Allocator, io: std.Io) !void {
    // these params control the amount + extent of per-frame external overlap + neighbour queries
    const ext_overlap_amount: f32 = 0.05; // number queries = 5% of number of entities
    const ext_overlap_scale: f32 = 0.05; // query radius is 5% of world extent
    const near_search_k: u8 = 5; // each neighbourhood search finds the 5 nearest entities
    const near_search_amount: f32 = 0.05; // number queries = 5% of number of entities
    const near_search_scale: f32 = 0.05; // max query radius = 5% of world extent
    // not parameterised, 100% of entities do overlap checks against other entities stored in tree
    var buf: [256]u8 = undefined;
    const params_str = try std.fmt.bufPrint(
        &buf,
        "ext_overlap: amount = {d:.3}, scale = {d:.3}\nnear_search: k = {d}, amount = {d:.3}, scale = {d:.3}\n",
        .{ ext_overlap_amount, ext_overlap_scale, near_search_k, near_search_amount, near_search_scale },
    );
    inline for (.{ Ball2f, Box2f }) |V| {
        std.debug.print(
            "\nRunning regular tree benchmarks for {d} {any}...\n{s}\n",
            .{ random_vols.getRandomBodies(V).len, V, params_str },
        );
        const RegIndexers = .{
            index.Indexer2f(2, 1, 3, .Morton),
            index.Indexer2f(2, 1, 4, .Morton),
            index.Indexer2f(2, 1, 5, .Morton),
            index.Indexer2f(2, 1, 6, .Morton),
            index.Indexer2f(2, 1, 7, .Morton),
            index.Indexer2f(4, 1, 1, .Morton),
            index.Indexer2f(4, 1, 2, .Morton),
            index.Indexer2f(4, 1, 3, .Morton),
            index.Indexer2f(4, 1, 1, .Zigzag),
            index.Indexer2f(4, 1, 2, .Zigzag),
            index.Indexer2f(4, 1, 3, .Zigzag),
        };
        inline for (RegIndexers) |Indexer| {
            try benchmarkTree(
                Indexer,
                st.SquareTree(Indexer, V, u32),
                allocator,
                io,
                ext_overlap_amount,
                ext_overlap_scale,
                near_search_k,
                near_search_amount,
                near_search_scale,
            );
        }
        std.debug.print(
            "\nRunning compressed tree benchmarks for {d} {any}..\n{s}\n",
            .{ random_vols.getRandomBodies(V).len, V, params_str },
        );
        const CompIndexers = .{
            index.Indexer2f(2, 2, 4, .Morton),
            index.Indexer2f(2, 3, 3, .Morton),
            index.Indexer2f(2, 4, 2, .Morton),
            index.Indexer2f(2, 5, 1, .Morton),
            index.Indexer2f(2, 6, 0, .Morton),
            index.Indexer2f(4, 2, 1, .Morton),
            index.Indexer2f(4, 2, 2, .Morton),
            index.Indexer2f(4, 3, 1, .Morton),
            index.Indexer2f(4, 3, 0, .Morton),
            index.Indexer2f(4, 2, 1, .Zigzag),
            index.Indexer2f(4, 3, 0, .Zigzag),
        };
        inline for (CompIndexers) |Indexer| {
            try benchmarkTree(
                Indexer,
                st.SquareTree(Indexer, V, u32),
                allocator,
                io,
                ext_overlap_amount,
                ext_overlap_scale,
                near_search_k,
                near_search_amount,
                near_search_scale,
            );
        }
    }
}

fn benchmarkTree(
    comptime Indexer: type,
    comptime TreeType: type,
    allocator: std.mem.Allocator,
    io: std.Io,
    ext_overlap_amount: f32,
    ext_overlap_scale: f32,
    near_search_k: u8,
    near_search_amount: f32,
    near_search_scale: f32,
) !void {
    const extent = 10.0;
    var tree = try TreeType.init(allocator, .{ 0, 0 }, .{ extent, extent }, max_capacity, 0);
    defer tree.deinit(allocator);
    const headers: [6][]const u8 = .{
        " add  ", " update ", " self-overlap ", " ext-overlap ", " neighbour ", " tick     ",
    };
    const formats: [6][]const u8 = .{
        " {d:>3.1}% ", " {d:>5.1}% ", " {d:>11.1}% ", " {d:>10.1}% ", " {d:>8.1}% ", " {d:>5.2} ms ",
    };
    var table = try DataTable(f64, 6, headers, formats).init(allocator, num_trials);
    defer table.deinit(allocator);
    const bodies = random_vols.getRandomBodies(TreeType.VolumeType);
    const pair_buf = try allocator.alloc(TreeType.OverlapPair, 1024 * bodies.len);
    defer allocator.free(pair_buf);
    var entity_indexes = try allocator.alloc(TreeType.ClientIdType, bodies.len);
    defer allocator.free(entity_indexes);
    for (0..entity_indexes.len) |i| entity_indexes[i] = @intCast(i);
    const ext_overlap_count = @max(1, @as(usize, @trunc(calc.asf32(bodies.len) * ext_overlap_amount)));
    const ext_query_ids = entity_indexes[0..ext_overlap_count];
    const ext_query_vols = try allocator.alloc(Ball2f, ext_overlap_count);
    defer allocator.free(ext_query_vols);
    var query_template = random_vols.balls.items[0];
    query_template.radius = extent * ext_overlap_scale;
    for (bodies[0..ext_overlap_count], ext_query_vols) |body, *query| {
        query.* = query_template;
        query.centre = body.getCentre();
    }
    const neighbour_count = @max(1, @as(usize, @trunc(calc.asf32(bodies.len) * near_search_amount)));
    const neighbour_range = extent * near_search_scale;
    const neighbour_k: usize = near_search_k;
    const neighbour_buf = try allocator.alloc(TreeType.Neighbour, neighbour_count * neighbour_k);
    defer allocator.free(neighbour_buf);
    const nbufs = try allocator.alloc([]TreeType.Neighbour, neighbour_count);
    defer allocator.free(nbufs);
    const neighbour_points = try allocator.alloc(Vec2f, neighbour_count);
    defer allocator.free(neighbour_points);
    const neighbour_excl_ids = try allocator.alloc(?TreeType.ClientIdType, neighbour_count);
    defer allocator.free(neighbour_excl_ids);
    for (bodies[0..neighbour_count], neighbour_points) |body, *point| point.* = body.getCentre();
    @memset(neighbour_excl_ids, null);

    // untimed warmup trials
    var n: usize = 0;
    for (0..untimed_trials) |_| {
        tree.clearStoredVolumes();
        try tree.addVolumes(bodies, entity_indexes);
        try tree.updateBoundsParallel(io);
        n += (try tree.findSelfOverlapsParallel(io, pair_buf)).len;
        for (nbufs, 0..) |*buf, i| buf.* = neighbour_buf[i * neighbour_k ..][0..neighbour_k];
        const neighbour_results = try tree.findNeighboursParallel(
            io,
            nbufs,
            neighbour_points,
            neighbour_excl_ids,
            near_search_k,
            neighbour_range,
        );
        for (neighbour_results) |result| n += result.len;
        n += (try tree.findExtOverlapsParallel(io, pair_buf, ext_query_ids, ext_query_vols)).len;
    }

    // timed trials
    var overlaps: usize = 0;
    var neighbours: usize = 0;
    var ext_overlaps: usize = 0;
    for (0..num_trials) |_| {
        const t_0 = timer.now(io);
        tree.clearStoredVolumes();
        try tree.addVolumes(bodies, entity_indexes);
        const t_1 = timer.now(io);
        try tree.updateBoundsParallel(io);
        const t_2 = timer.now(io);
        overlaps = (try tree.findSelfOverlapsParallel(io, pair_buf)).len;
        const t_3 = timer.now(io);
        neighbours = 0;
        for (nbufs, 0..) |*buf, i| buf.* = neighbour_buf[i * neighbour_k ..][0..neighbour_k];
        const neighbour_results = try tree.findNeighboursParallel(
            io,
            nbufs,
            neighbour_points,
            neighbour_excl_ids,
            near_search_k,
            neighbour_range,
        );
        for (neighbour_results) |result| neighbours += result.len;
        const t_4 = timer.now(io);
        ext_overlaps = (try tree.findExtOverlapsParallel(io, pair_buf, ext_query_ids, ext_query_vols)).len;
        const t_5 = timer.now(io);
        const total_ns = elapsedNs(t_0, t_5);
        try table.addRow(.{
            100 * elapsedNs(t_0, t_1) / total_ns,
            100 * elapsedNs(t_1, t_2) / total_ns,
            100 * elapsedNs(t_2, t_3) / total_ns,
            100 * elapsedNs(t_4, t_5) / total_ns,
            100 * elapsedNs(t_3, t_4) / total_ns,
            total_ns / 1_000_000,
        });
    }

    const max_occ = try tree.getMaxLeafOccupancy();
    const stats_str = try table.getStatsTable(allocator);
    defer allocator.free(stats_str);
    std.debug.print(
        "{s}:\nmax leaf {}, overlaps {}, neighbours {}, ext-overlaps {}, size {}B\n{s}\n",
        .{ Indexer.type_label, max_occ, overlaps, neighbours, ext_overlaps, @sizeOf(TreeType), stats_str },
    );
}

/// Stores several columns of same-typed data and provides helpers for computing stats + printing.
pub fn DataTable(
    comptime T: type,
    comptime num_cols: u8,
    comptime headers: [num_cols][]const u8,
    comptime formats: [num_cols][]const u8,
) type {
    return struct {
        column_data: [num_cols]std.ArrayList(T) = undefined,
        num_rows: usize = 0,
        const Self = @This();

        pub fn init(allocator: std.mem.Allocator, capacity: usize) !Self {
            var cols: [num_cols]std.ArrayList(T) = undefined;
            var cols_created: usize = 0;
            errdefer for (0..cols_created) |i| cols[i].deinit(allocator);
            for (0..num_cols) |i| {
                cols[i] = try std.ArrayList(T).initCapacity(allocator, capacity);
                cols_created += 1;
            }
            return .{ .column_data = cols };
        }

        pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
            for (0..num_cols) |i| self.column_data[i].deinit(allocator);
        }

        /// Appends a row; returns OutOfMemory if a column is at capacity.
        pub fn addRow(self: *Self, vals: [num_cols]T) !void {
            for (0..num_cols) |j| try self.column_data[j].appendBounded(vals[j]);
            self.num_rows += 1;
        }

        /// Clears columns' contents without releasing their backing memory.
        pub fn clear(self: *Self) void {
            for (0..num_cols) |j| self.column_data[j].clearRetainingCapacity();
            self.num_rows = 0;
        }

        /// Sorts column data and gets range + IQR stats for each: { min, q1, q2, q3, max }.
        /// Doesn't interpolate between indexes: inacurate for a low sample sizes.
        pub fn computeStats(self: *Self) ?[num_cols][5]T {
            if (self.column_data[0].items.len == 0) return null;
            var col_stats: [num_cols][5]T = undefined;
            for (0..num_cols) |j| {
                const items = self.column_data[j].items;
                std.sort.pdq(T, items, {}, std.sort.asc(T));
                const min = items[0];
                const q1 = items[1 * items.len / 4];
                const q2 = items[2 * items.len / 4];
                const q3 = items[3 * items.len / 4];
                const max = items[items.len - 1];
                col_stats[j] = .{ min, q1, q2, q3, max };
            }
            return col_stats;
        }

        /// Builds a multi-line string representing a table's stats.
        /// Caller owns the returned slice.
        pub fn getStatsTable(self: *Self, allocator: std.mem.Allocator) ![]u8 {
            var string_list = try std.ArrayList(u8).initCapacity(allocator, @as(usize, num_cols) * 64);
            errdefer string_list.deinit(allocator);
            const col_stats = self.computeStats() orelse return error.NoValues;
            try string_list.appendSlice(allocator, "|     |");
            for (0..num_cols) |j| {
                try string_list.appendSlice(allocator, headers[j]);
                try string_list.append(allocator, '|');
            }
            const row_titles: [5][]const u8 = .{ " min ", " q1  ", " q2  ", " q3  ", " max " };
            var stat_buff: [32]u8 = undefined;
            for (0..5) |i| {
                try string_list.appendSlice(allocator, "\n|");
                try string_list.appendSlice(allocator, row_titles[i]);
                try string_list.append(allocator, '|');
                inline for (0..num_cols) |j| {
                    const cell_val = col_stats[j][i];
                    const stat_str = try std.fmt.bufPrint(&stat_buff, formats[j], .{cell_val});
                    try string_list.appendSlice(allocator, stat_str);
                    try string_list.append(allocator, '|');
                }
            }
            try string_list.append(allocator, '\n');
            return string_list.toOwnedSlice(allocator);
        }
    };
}
