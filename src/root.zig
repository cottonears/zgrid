pub const calc = @import("maths/calc.zig");
pub const curve = @import("maths/curve.zig");
pub const index = @import("maths/index.zig");
pub const rand = @import("maths/rand.zig");
pub const volume = @import("maths/volume.zig");
pub const square_tree = @import("square_tree.zig");
pub const Ball2f = volume.Ball2f;
pub const Box2f = volume.Box2f;
pub const Curve = curve.Curve;
pub const Line2f = volume.Line2f;
pub const OrientedBox2f = volume.OrientedBox2f;
pub const Vec2f = calc.Vec2f;

// ----------------------------------------------------------------------------
// Module-level tests that use random volumes and / or write output files.
// ----------------------------------------------------------------------------
const std = @import("std");
const draw = @import("draw.zig");
const math = std.math;
const testing = std.testing;

const test_alloc = testing.allocator;
const test_dir = "test-out";
const test_num_volumes = 16000;
const test_min_extent = 0.1;
const test_max_extent = 5.0;
const test_space_min = -50;
const test_space_max = 50;

test "tree self overlaps matches brute force" {
    const Trees = .{
        square_tree.SquareTree(index.Indexer2f(.Spring16, 1), Ball2f, u16),
        square_tree.SquareTree(index.Indexer2f(.Zigzag16, 1), Ball2f, u16),
        square_tree.SquareTree(index.Indexer2f(.Morton16, 1), OrientedBox2f, u16),
        square_tree.SquareTree(index.Indexer2f(.Spring64, 1), Ball2f, u16),
        square_tree.SquareTree(index.Indexer2f(.Zigzag64, 1), OrientedBox2f, u16),
        square_tree.SquareTree(index.Indexer2f(.Morton64, 1), Ball2f, u16),
    };
    const num_vols = 200;
    const seed = rand.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer rand.printErrorMessageForRandomSeed(seed);
    var test_vols = try rand.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_vols,
        .{ .uniform = .{ .min = 0.005, .max = 0.04 } },
        .{ .uniform = .{ .min = 0.05, .max = 0.95 } },
    );
    defer test_vols.deinit(test_alloc);
    // generate random volumes and check for overlaps between all pairs
    inline for (Trees) |Tree| {
        const bodies = test_vols.getVolumes(Tree.VolumeType);
        var tree = try Tree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, num_vols, 0);
        defer tree.deinit(test_alloc);
        const indexes = calc.getRange(u16, num_vols);
        try tree.addVolumes(bodies, &indexes);
        try tree.updateBoundsParallel(testing.io);
        var expected: std.ArrayList([2]u16) = .empty;
        defer expected.deinit(test_alloc);
        for (bodies, 0..) |a, i| {
            for (bodies[i + 1 ..], i + 1..) |b, j| {
                if (volume.checkVolumesOverlap(a, b)) {
                    try expected.append(test_alloc, .{ @intCast(i), @intCast(j) });
                }
            }
        }
        // check that the pairwise overlap results agree with those returned by the tree's method
        const found_buff = try test_alloc.alloc([2]u16, num_vols * num_vols);
        defer test_alloc.free(found_buff);
        const actual = try tree.findSelfOverlapsParallel(testing.io, found_buff);
        calc.sortPairsLexicographic(u16, expected.items);
        calc.sortPairsLexicographic(u16, actual);
        try testing.expectEqualSlices([2]u16, expected.items, actual);
    }
}

test "tree neighbours matches brute force" {
    const num_vols = 200;
    const seed = rand.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer rand.printErrorMessageForRandomSeed(seed);
    const Tree = square_tree.SquareTree(index.Indexer2f(.Zigzag64, 1), Ball2f, u16);
    var tree = try Tree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, num_vols, 0);
    defer tree.deinit(test_alloc);
    var test_vols = try rand.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_vols,
        .{ .uniform = .{ .min = 0.005, .max = 0.04 } },
        .{ .uniform = .{ .min = 0.05, .max = 0.95 } },
    );
    defer test_vols.deinit(test_alloc);
    const boxes = test_vols.getVolumes(Ball2f);
    const indexes = calc.getRange(Tree.ClientIdType, num_vols);
    try tree.addVolumes(boxes, &indexes);
    try tree.updateBounds();
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

test "draw trees" {
    // Not an automated test: eyeball the written files to verify
    const Indexer = index.Indexer2f(.Spring16, 1);
    const Trees = .{
        square_tree.SquareTree(Indexer, Ball2f, u16),
        square_tree.SquareTree(Indexer, Box2f, u16),
        square_tree.SquareTree(Indexer, OrientedBox2f, u16),
    };
    const num_vols = 32;
    const seed = rand.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer rand.printErrorMessageForRandomSeed(seed);
    var test_vols = try rand.TestVolumes.initRandom(
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
        const bodies = test_vols.getVolumes(Tree.VolumeType);
        const indexes = calc.getRange(u16, num_vols);
        try tree.addVolumes(bodies, &indexes);
        try tree.updateBounds();
        var canvas = try tree.drawTreeSvg(test_alloc, true);
        defer canvas.deinit(test_alloc);
        var buf: [512]u8 = undefined;
        const path = try std.fmt.bufPrint(&buf, "{s}/{s}.html", .{ test_dir, @typeName(Tree) });
        try canvas.writeHtml(test_alloc, testing.io, path, true);
    }
}

test "draw indexer curves" {
    const write_line_length = false;
    const Indexers = .{
        index.Indexer2f(Curve.Morton16, 2),
        index.Indexer2f(Curve.Spring16, 1),
        index.Indexer2f(Curve.Zigzag16, 1),
    };
    const bg_style: draw.Style = .{ .fill_active = true, .fill_hsl = .{ 0, 0, 95 } };
    const line_style: draw.Style = .{ .stroke_width = 2, .stroke_hsl = .{ 90, 60, 40 } };
    const min_pt = Vec2f{ 0, 0 };
    const max_pt = Vec2f{ 1024, 1024 };
    // draw some pretty pictures of the indexers' curves so they can be eyeballed.
    inline for (Indexers) |Indexer| {
        const idx = try Indexer.init(min_pt, max_pt);
        var pts: [Indexer.num_leaves]Vec2f = undefined;
        var curve_lenth: f32 = 0.0;
        for (0..pts.len) |i| {
            const cell = idx.getLeafCellBoundary(@truncate(i));
            pts[i] = cell.getCentre();
            if (i > 0) {
                curve_lenth += calc.norm(pts[i] - pts[i - 1]);
            }
        }
        var test_canvas = try draw.SvgCanvas.init(test_alloc, min_pt, max_pt, bg_style);
        defer test_canvas.deinit(test_alloc);
        try test_canvas.addPolyline(test_alloc, &pts, line_style);
        var sbuff: [128]u8 = undefined;
        if (write_line_length) {
            const length_str = try std.fmt.bufPrint(&sbuff, "length = {d:.1}\n", .{curve_lenth});
            const text_loc = min_pt + calc.scaledVec(0.5, max_pt - min_pt);
            try test_canvas.addText(test_alloc, text_loc, length_str, 20, .{ 0, 0, 0 });
        }
        const fpath = try std.fmt.bufPrint(&sbuff, "{s}/{any}.html", .{ test_dir, Indexer });
        try test_canvas.writeHtml(test_alloc, testing.io, fpath, true);
        // operator should inspect the output: expect("looks good to me")
    }
}
