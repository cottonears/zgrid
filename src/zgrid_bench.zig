const std = @import("std");
const builtin = @import("builtin");
const zgrid = @import("zgrid");
const calc = zgrid.calc;
const stats = zgrid.stats;
const volume = zgrid.volume;
const timer = std.Io.Clock.awake;
const Allocator = std.mem.Allocator;
const ArgsIter = std.process.Args.Iterator;
const Vec2f = zgrid.Vec2f;
const Ball2f = zgrid.Ball2f;
const Box2f = zgrid.Box2f;
const Line2f = zgrid.Line2f;
const OrientedBox2f = zgrid.OrientedBox2f;
const DataTable = zgrid.stats.DataTable;
const ProbDensityFunc = zgrid.stats.ProbDensityFunc;
const TestVolumes = zgrid.stats.TestVolumes;
const Indexer2f = zgrid.Indexer2f;
const SquareTree = zgrid.SquareTree;

const max_capacity = 400_000;
const min_trials = 20;
const min_num_vols = 100;
const stat_header = "percentile";
const stat_percentiles: [2]u8 = .{ 50, 95 };
const untimed_trials = 5;
const usage_msg =
    \\Usage: zgrid-bench [options]
    \\  -i: Set an input file (csv or txt) to load test volumes from (see readme for correct format)
    \\  -t: Number of times to repeat each benchmark (for stable timing averages); default = 100
    \\
;
var input_file: ?[]const u8 = null;
var output_dir: ?[]const u8 = null;
var random_vols: TestVolumes = undefined;
var num_trials: u8 = 100;

pub fn main(init: std.process.Init) !void {
    const allocator = init.arena.allocator();
    var args_iter = try ArgsIter.initAllocator(init.minimal.args, allocator);
    defer args_iter.deinit();
    _ = args_iter.next(); // skip the program name
    try processArgs(init.io, &args_iter);

    if (input_file) |file| {
        random_vols = try TestVolumes.initCsv(allocator, init.io, file);
        std.debug.print(
            "Running benchmarks using volumes from '{s}' (trials = {})...\n",
            .{ file, num_trials },
        );
    } else { // default built-in benchmark
        const num_vols = 20_000;
        const pos_dist: ProbDensityFunc = .{ .normal = .{ .mean = 5.0, .stddev = 1.5 } };
        const size_dist: ProbDensityFunc = .{ .uniform = .{ .min = 0.001, .max = 0.05 } };
        var prng = std.Random.DefaultPrng.init(0);
        random_vols = try TestVolumes.initRandom(allocator, prng.random(), num_vols, size_dist, pos_dist);
        std.debug.print(
            "Running benchmarks for {} random vols (trials = {})...\n\n",
            .{ num_vols, num_trials },
        );
    }
    defer random_vols.deinit(allocator);

    try benchmarkOverlapChecks(allocator, init.io);
    try benchmarkIndexing(allocator, init.io);
    try benchmarkSquareTrees(allocator, init.io);
}

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
        } else {
            std.debug.print("Unrecognised argument '{s}'.\n{s}", .{ arg, usage_msg });
            return error.UnrecognisedArgument;
        }
    }
}

fn benchmarkOverlapChecks(allocator: Allocator, io: std.Io) !void {
    const num_cols = 5;
    const headers: [num_cols][]const u8 = .{
        " ball-ball ", " box-box ", " ball-box ", " obb-box ", " line-box ",
    };
    const formats: [num_cols][]const u8 = .{
        " {d:>9.3} ", " {d:>7.3} ", " {d:>8.3} ", " {d:>7.3} ", " {d:>8.3} ",
    };
    const OverlapTable = DataTable(f64, num_cols, "| {s:<10} |", headers, formats);
    var table = try OverlapTable.init(allocator, num_trials);
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
        for (random_vols.balls.items) |b| n += if (volume.checkVolumesOverlap(first_ball, b)) 1 else 0;
        for (random_vols.boxes.items) |b| n += if (volume.checkVolumesOverlap(query_box, b)) 1 else 0;
        for (random_vols.balls.items) |b| n += if (volume.checkVolumesOverlap(query_box, b)) 1 else 0;
        for (random_vols.boxes.items) |b| n += if (volume.checkVolumesOverlap(first_ball, b)) 1 else 0;
        for (random_vols.boxes.items) |b| n += if (volume.checkVolumesOverlap(query_obb, b)) 1 else 0;
        for (random_vols.boxes.items) |b| n += if (volume.checkVolumesOverlap(query_line, b)) 1 else 0;
    }

    // timed trials
    var overlap_count: u32 = 0;
    const ball_checks: f64 = @floatFromInt(random_vols.balls.items.len);
    const box_checks: f64 = @floatFromInt(random_vols.boxes.items.len);
    const mixed_checks: f64 = @floatFromInt(random_vols.balls.items.len + random_vols.boxes.items.len);
    for (0..num_trials) |_| {
        const t_0 = timer.now(io);
        for (random_vols.balls.items) |b| {
            overlap_count += if (volume.checkVolumesOverlap(first_ball, b)) 1 else 0;
        }
        const t_1 = timer.now(io);
        for (random_vols.boxes.items) |b| {
            overlap_count += if (volume.checkVolumesOverlap(query_box, b)) 1 else 0;
        }
        const t_2 = timer.now(io);
        for (random_vols.balls.items) |b| {
            overlap_count += if (volume.checkVolumesOverlap(query_box, b)) 1 else 0;
        }
        for (random_vols.boxes.items) |b| {
            overlap_count += if (volume.checkVolumesOverlap(first_ball, b)) 1 else 0;
        }
        const t_3 = timer.now(io);
        for (random_vols.boxes.items) |b| {
            overlap_count += if (volume.checkVolumesOverlap(query_obb, b)) 1 else 0;
        }
        const t_4 = timer.now(io);
        for (random_vols.boxes.items) |b| {
            overlap_count += if (volume.checkVolumesOverlap(query_line, b)) 1 else 0;
        }
        const t_5 = timer.now(io);
        table.addRow(.{
            stats.elapsedNs(t_0, t_1) / ball_checks,
            stats.elapsedNs(t_1, t_2) / box_checks,
            stats.elapsedNs(t_2, t_3) / mixed_checks,
            stats.elapsedNs(t_3, t_4) / box_checks,
            stats.elapsedNs(t_4, t_5) / box_checks,
        });
    }

    std.debug.print("Overlap checks: found {} overlaps:\n", .{overlap_count});
    const left_header = "percentile";
    const stats_str = try table.getStatsTable(allocator, left_header[0..], &stat_percentiles);
    defer allocator.free(stats_str);
    std.debug.print("{s}\n", .{stats_str});
}

fn benchmarkIndexing(allocator: Allocator, io: std.Io) !void {
    const IndexerTypes = [_]type{
        Indexer2f(.Morton16, 1),
        Indexer2f(.Zigzag16, 1),
        Indexer2f(.Morton32, 1),
        Indexer2f(.Morton64, 1),
        Indexer2f(.Zigzag64, 1),
        Indexer2f(.Morton128, 1),
        Indexer2f(.Morton256, 1),
        Indexer2f(.Zigzag256, 1),
    };
    const headers: [2][]const u8 = .{ " time (ns/pt) ", " inter-leaf dist " };
    const formats: [2][]const u8 = .{ " {d:>12.3} ", " {d:>15.4} " };
    const IndexingTable = DataTable(f64, 2, "| {s:<24} |", headers, formats);
    var table = try IndexingTable.init(allocator, num_trials);
    defer table.deinit(allocator);
    var table_str = try std.ArrayList(u8).initCapacity(allocator, 2048);
    defer table_str.deinit(allocator);

    inline for (IndexerTypes) |Indexer| {
        table.clear();
        const pt1 = random_vols.balls.items[0].centre;
        const pt2 = random_vols.balls.items[1].centre;
        var indexer = try Indexer.init(pt1, pt2);
        const indexes = try allocator.alloc(Indexer.CurveIndex, random_vols.boxes.items.len);
        defer allocator.free(indexes);

        // measure inter-leaf distances (crude indicator of how well the curve preserves locality)
        var index_dist_sum: f64 = 0.0;
        var next_centre = indexer.getLeafCellBoundary(0).getCentre();
        for (1..Indexer.num_leaves) |i| {
            const centre = indexer.getLeafCellBoundary(@truncate(i)).getCentre();
            index_dist_sum += calc.norm(centre - next_centre);
            next_centre = centre;
        }
        const avg_il_dist = index_dist_sum / @as(f64, @floatFromInt(Indexer.num_leaves - 1));

        // untimed warmup trials
        for (0..untimed_trials) |_| {
            for (random_vols.boxes.items, 0..) |b, i| {
                indexes[i] = indexer.getLeafIndexForPoint(b.getCentre());
            }
        }

        // timed trials
        for (0..num_trials) |_| {
            const t_0 = timer.now(io);
            for (random_vols.boxes.items, 0..) |b, i| {
                indexes[i] = indexer.getLeafIndexForPoint(b.getCentre());
            }
            const t_1 = timer.now(io);
            const avg_t = stats.elapsedNs(t_0, t_1) / @as(f64, @floatFromInt(indexes.len));
            table.addRow(.{ avg_t, avg_il_dist });
        }

        if (table_str.items.len == 0) {
            try table.appendHeader(allocator, &table_str, "indexer");
        }
        try table.appendStatsRow(allocator, &table_str, Indexer.type_label, stat_percentiles[0]);
    }
    std.debug.print("Indexing benchmark (p{}):\n{s}\n", .{ stat_percentiles[0], table_str.items });
}

fn benchmarkSquareTrees(allocator: Allocator, io: std.Io) !void {
    // these params control the amount + extent of per-frame external overlap + neighbour queries
    const ext_overlap_amount: f32 = 0.05; // number queries = 5% of number of vols
    const ext_overlap_scale: f32 = 0.05; // query radius is 5% of world extent
    const near_search_k: u8 = 5; // each neighbourhood search finds the 5 nearest vols
    const near_search_amount: f32 = 0.05; // number queries = 5% of number of vols
    const near_search_scale: f32 = 0.05; // max query radius = 5% of world extent
    // NOTE: findSelfOverlaps always checks 100% all stored volumes
    var buf: [256]u8 = undefined;
    const params_str = try std.fmt.bufPrint(
        &buf,
        "ext_overlap: amount = {d:.3}, scale = {d:.3}\nnear_search: k = {d}, amount = {d:.3}, scale = {d:.3}",
        .{ ext_overlap_amount, ext_overlap_scale, near_search_k, near_search_amount, near_search_scale },
    );
    inline for (.{ Ball2f, Box2f }) |V| {
        std.debug.print("\nUncompressed tree benchmarks for {any}...\n{s}\n", .{ V, params_str });
        var reg_table_str = try std.ArrayList(u8).initCapacity(allocator, 4096);
        defer reg_table_str.deinit(allocator);
        const RegIndexers = .{
            Indexer2f(.Morton16, 1),
            Indexer2f(.Spring16, 1),
            Indexer2f(.Zigzag16, 1),
            Indexer2f(.Morton32, 1),
            Indexer2f(.Morton64, 1),
            Indexer2f(.Spring64, 1),
            Indexer2f(.Zigzag64, 1),
            Indexer2f(.Morton128, 1),
            Indexer2f(.Morton256, 1),
            Indexer2f(.Spring256, 1),
            Indexer2f(.Zigzag256, 1),
        };
        inline for (RegIndexers) |Indexer| {
            try benchmarkTree(
                Indexer,
                SquareTree(Indexer, V, u32),
                allocator,
                io,
                &reg_table_str,
                ext_overlap_amount,
                ext_overlap_scale,
                near_search_k,
                near_search_amount,
                near_search_scale,
                stat_percentiles[0],
            );
        }
        std.debug.print("{s}\n", .{reg_table_str.items});

        std.debug.print("\nCompressed tree benchmarks for {any}:\n{s}\n", .{ V, params_str });
        const CompIndexers = .{
            Indexer2f(.Morton64, 2),
            Indexer2f(.Morton64, 4),
            Indexer2f(.Spring64, 2),
            Indexer2f(.Zigzag64, 2),
            Indexer2f(.Morton64, 6),
            Indexer2f(.Spring64, 3),
            Indexer2f(.Zigzag64, 3),
        };
        var comp_table_str = try std.ArrayList(u8).initCapacity(allocator, 4096);
        defer comp_table_str.deinit(allocator);
        inline for (CompIndexers) |Indexer| {
            try benchmarkTree(
                Indexer,
                SquareTree(Indexer, V, u32),
                allocator,
                io,
                &comp_table_str,
                ext_overlap_amount,
                ext_overlap_scale,
                near_search_k,
                near_search_amount,
                near_search_scale,
                stat_percentiles[0],
            );
        }
        std.debug.print("{s}\n", .{comp_table_str.items});
    }
}

fn benchmarkTree(
    comptime Indexer: type,
    comptime TreeType: type,
    allocator: Allocator,
    io: std.Io,
    table_str: *std.ArrayList(u8),
    ext_overlap_amount: f32,
    ext_overlap_scale: f32,
    near_search_k: u8,
    near_search_amount: f32,
    near_search_scale: f32,
    percentile: u8,
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

    const TreeTable = DataTable(f64, 6, "| {s:<24} |", headers, formats);
    var table = try TreeTable.init(allocator, num_trials);
    defer table.deinit(allocator);
    const bodies = random_vols.getVolumes(TreeType.Volume);
    const pair_buf = try allocator.alloc([2]TreeType.ClientId, 1024 * bodies.len);
    defer allocator.free(pair_buf);
    var entity_indexes = try allocator.alloc(TreeType.ClientId, bodies.len);
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
    const neighbour_excl_ids = try allocator.alloc(?TreeType.ClientId, neighbour_count);
    defer allocator.free(neighbour_excl_ids);
    for (bodies[0..neighbour_count], neighbour_points) |body, *point| point.* = body.getCentre();
    @memset(neighbour_excl_ids, null);

    // untimed warmup trials
    var n: usize = 0;
    for (0..untimed_trials) |_| {
        tree.clear();
        try tree.addVolumes(bodies, entity_indexes);
        try tree.buildParallel(io);
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
        tree.clear();
        try tree.addVolumes(bodies, entity_indexes);
        const t_1 = timer.now(io);
        try tree.buildParallel(io);
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
        const total_ms = stats.elapsedMs(t_0, t_5);
        table.addRow(.{
            100 * stats.elapsedMs(t_0, t_1) / total_ms,
            100 * stats.elapsedMs(t_1, t_2) / total_ms,
            100 * stats.elapsedMs(t_2, t_3) / total_ms,
            100 * stats.elapsedMs(t_4, t_5) / total_ms,
            100 * stats.elapsedMs(t_3, t_4) / total_ms,
            total_ms,
        });
    }

    if (table_str.items.len == 0) {
        try table.appendHeader(allocator, table_str, "indexer");
    }
    try table.appendStatsRow(allocator, table_str, Indexer.type_label, percentile);

    // const times = try tree.time_stats.getStatsTable(allocator, "step", &stat_percentiles);
    // defer allocator.free(times);
    // std.debug.print("{s}:\n{s}\n", .{ Indexer.type_label, times });
}
