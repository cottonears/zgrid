pub const calc = @import("maths/calc.zig");
pub const curve = @import("maths/curve.zig");
pub const index = @import("maths/index.zig");
pub const stats = @import("maths/stats.zig");
pub const volume = @import("maths/volume.zig");
pub const parallel = @import("parallel.zig");
pub const square_tree = @import("square_tree.zig");
pub const svg = @import("svg.zig");
const std = @import("std");

pub const Vec2f = volume.Vec2f;
pub const Curve = curve.Curve;
pub const Ball2f = volume.Ball2f;
pub const Box2f = volume.Box2f;
pub const Line2f = volume.Line2f;
pub const OrientedBox2f = volume.OrientedBox2f;
pub const Indexer2f = index.Indexer2f;
pub const SquareTree = square_tree.SquareTree;

/// Draws a tree's grid subdivisions, cell labels, and stored volumes to an svg file.
/// Accepts a pointer to any tree exposing the same public interface as `SquareTree`.
/// NOTE: tightly coupled to square tree at the moment: will need work when another tree is added.
pub fn writeTreeSvg(
    T: type,
    tree: *T,
    io: std.Io,
    allocator: std.mem.Allocator,
    filename: []const u8,
    show_client_ids: bool,
    wrap_html: bool,
) !void {
    const bg: svg.Style = .{
        .fill_active = true,
        .fill_hsl = .{ 0, 0, 95 },
        .stroke_active = false,
    };
    var canvas = try svg.Canvas.init(allocator, tree.indexer.min_pt, tree.indexer.max_pt, bg);
    defer canvas.deinit(allocator);
    const extent = tree.indexer.max_pt - tree.indexer.min_pt;
    const scale = @reduce(.Max, extent) / 800.0;
    // draw grid subdivisions + cell labels, finest level first
    const palette = [_][3]u16{
        .{ 10, 60, 60 },
        .{ 80, 60, 60 },
        .{ 150, 60, 50 },
        .{ 200, 60, 60 },
        .{ 270, 60, 60 },
        .{ 335, 60, 60 },
    };
    var label_buf: [16]u8 = undefined;
    for (0..T.depth) |lvl_offset| {
        const lvl = T.depth - lvl_offset - 1;
        const style: svg.Style = .{
            .stroke_hsl = palette[lvl % palette.len],
            .stroke_width = scale * calc.asf32(lvl_offset),
        };
        const font_size: f32 = scale * (4.0 + 8.0 * calc.asf32(lvl_offset + 1));
        for (0..T.nodes_in_level[lvl]) |i| {
            const node_index: T.CurveIndex = @intCast(i);
            const cell = tree.indexer.getCellBoundaryAtLevel(@intCast(lvl), node_index);
            const label_width = (std.math.log2_int(usize, T.nodes_in_level[lvl]) + 3) / 4;
            const label = try std.fmt.bufPrint(&label_buf, "{X:0>[1]}", .{ node_index, label_width });
            try canvas.addRectangle(allocator, cell.min, cell.max, style);
            try canvas.addText(allocator, cell.getCentre(), label, font_size, style.stroke_hsl);
        }
    }
    // find every client id that participates in an overlap
    const volumes = tree.leaf_vols[0..tree.num_volumes];
    const ids = tree.leaf_ids[0..tree.num_volumes];
    const overlap_buf = try allocator.alloc([2]T.ClientId, 16 * volumes.len);
    defer allocator.free(overlap_buf);
    const pairs = try tree.findSelfOverlaps(overlap_buf);
    var overlapping = std.AutoHashMap(T.ClientId, void).init(allocator);
    defer overlapping.deinit();
    for (pairs) |pair| {
        try overlapping.put(pair[0], {});
        try overlapping.put(pair[1], {});
    }
    //draw the stored volumes, colouring overlapping ones differently
    const default_style: svg.Style = .{ .stroke_hsl = .{ 0, 0, 30 }, .stroke_width = 1.5 * scale };
    const overlap_style: svg.Style = .{ .stroke_hsl = .{ 0, 50, 50 }, .stroke_width = 2.0 * scale };
    const id_label_hsl: [3]u16 = .{ 0, 0, 0 };
    const id_label_font_size: f32 = scale * 10.0;
    var id_label_buf: [20]u8 = undefined;
    for (volumes, ids) |v, id| {
        const style = if (overlapping.contains(id)) overlap_style else default_style;
        if (T.Volume == Ball2f) {
            try canvas.addCircle(allocator, v.centre, v.radius, style);
        } else if (T.Volume == Box2f) {
            try canvas.addRectangle(allocator, v.min, v.max, style);
        } else if (T.Volume == OrientedBox2f) {
            var corners = v.getCorners();
            try canvas.addPolygon(allocator, &corners, style);
        } else {
            @compileError("drawTreeSvg: unsupported volume type " ++ @typeName(T.Volume));
        }
        if (show_client_ids) {
            const label = try std.fmt.bufPrint(&id_label_buf, "{d}", .{id});
            try canvas.addText(allocator, v.getCentre(), label, id_label_font_size, id_label_hsl);
        }
    }

    if (std.fs.path.dirname(filename)) |dir| {
        try std.Io.Dir.cwd().createDirPath(io, dir);
    }
    try canvas.writeToFile(io, test_alloc, filename, wrap_html);
}

// ----------------------------------------------------------------------------
// Module-level tests that use random volumes and / or write output files.
// ----------------------------------------------------------------------------
const testing = std.testing;
const test_alloc = testing.allocator;
const test_dir = "test-out";
const num_volumes = 1_000;
const min_extent = 0.1;
const max_extent = 5.0;
const space_min = -50;
const space_max = 50;

test "tree self overlaps matches brute force" {
    const Trees = .{
        SquareTree(Indexer2f(.Morton16, 1), OrientedBox2f, u16),
        SquareTree(Indexer2f(.Spring16, 1), Ball2f, u16),
        SquareTree(Indexer2f(.Zigzag16, 1), Box2f, u16),
        SquareTree(Indexer2f(.Morton64, 1), Ball2f, u16),
        SquareTree(Indexer2f(.Spring64, 1), Box2f, u16),
        SquareTree(Indexer2f(.Zigzag64, 1), OrientedBox2f, u16),
    };
    const seed = stats.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer stats.printErrorMessageForRandomSeed(seed);
    var test_vols = try stats.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_volumes,
        .{ .uniform = .{ .min = 0.005, .max = 0.04 } },
        .{ .uniform = .{ .min = 0.05, .max = 0.95 } },
    );
    defer test_vols.deinit(test_alloc);
    // generate random volumes and check for overlaps between all pairs
    inline for (Trees) |Tree| {
        const vols = test_vols.getVolumes(Tree.Volume);
        var tree = try Tree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, num_volumes, 0);
        defer tree.deinit(test_alloc);
        const indexes = calc.getRange(u16, num_volumes);
        try tree.addVolumes(vols, &indexes);
        try tree.buildParallel(testing.io);
        var expected: std.ArrayList([2]u16) = .empty;
        defer expected.deinit(test_alloc);
        for (vols, 0..) |a, i| {
            for (vols[i + 1 ..], i + 1..) |b, j| {
                if (volume.checkVolumesOverlap(a, b)) {
                    try expected.append(test_alloc, .{ @intCast(i), @intCast(j) });
                }
            }
        }
        // check that the pairwise overlap results agree with those returned by the tree's method
        const found_buf = try test_alloc.alloc([2]u16, num_volumes * num_volumes / 2);
        defer test_alloc.free(found_buf);
        const actual = try tree.findSelfOverlapsParallel(testing.io, found_buf);
        calc.sortPairsLexicographic(u16, expected.items);
        calc.sortPairsLexicographic(u16, actual);
        try testing.expectEqualSlices([2]u16, expected.items, actual);
    }
}

test "tree neighbours matches brute force" {
    const seed = stats.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer stats.printErrorMessageForRandomSeed(seed);
    const Tree = SquareTree(index.Indexer2f(.Zigzag64, 1), Ball2f, u16);
    var tree = try Tree.init(test_alloc, .{ 0, 0 }, .{ 1, 1 }, num_volumes, 0);
    defer tree.deinit(test_alloc);
    var test_vols = try stats.TestVolumes.initRandom(
        test_alloc,
        prng.random(),
        num_volumes,
        .{ .uniform = .{ .min = 0.005, .max = 0.04 } },
        .{ .uniform = .{ .min = 0.05, .max = 0.95 } },
    );
    defer test_vols.deinit(test_alloc);
    const boxes = test_vols.getVolumes(Ball2f);
    const indexes = calc.getRange(Tree.ClientId, num_volumes);
    try tree.addVolumes(boxes, &indexes);
    try tree.build();
    // check for closest neighbours between all pairs
    var expected: [num_volumes][3]Tree.Neighbour = undefined;
    var neighbour_storage: [num_volumes][3]Tree.Neighbour = undefined;
    var bufs: [num_volumes][]Tree.Neighbour = undefined;
    var points: [num_volumes]Vec2f = undefined;
    var excl_ids: [num_volumes]?Tree.ClientId = undefined;
    for (boxes, indexes, 0..) |box_a, id_a, i| {
        var nearest = ([1]Tree.Neighbour{.{ .id = 0, .dist = std.math.floatMax(f32) }}) ** 3;
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
    try testing.expectEqual(num_volumes, results.len);
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
        SquareTree(Indexer, Ball2f, u16),
        SquareTree(Indexer, Box2f, u16),
        SquareTree(Indexer, OrientedBox2f, u16),
    };
    const num_vols = 32;
    const seed = stats.getClockBasedRngSeed(testing.io);
    var prng = std.Random.DefaultPrng.init(seed);
    errdefer stats.printErrorMessageForRandomSeed(seed);
    var test_vols = try stats.TestVolumes.initRandom(
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
    inline for (Trees) |T| {
        var tree = try T.init(test_alloc, min_pt, max_pt, num_vols, 0);
        defer tree.deinit(test_alloc);
        const bodies = test_vols.getVolumes(T.Volume);
        const indexes = calc.getRange(u16, num_vols);
        try tree.addVolumes(bodies, &indexes);
        try tree.build();

        var buf: [512]u8 = undefined;
        const path = try std.fmt.bufPrint(&buf, "{s}/{s}.html", .{ test_dir, @typeName(T) });
        try writeTreeSvg(
            T,
            &tree,
            testing.io,
            test_alloc,
            path,
            false,
            true,
        );
    }
}

test "draw indexer curves" {
    const write_line_length = false;
    const Indexers = .{
        index.Indexer2f(Curve.Morton16, 2),
        index.Indexer2f(Curve.Spring16, 1),
        index.Indexer2f(Curve.Zigzag16, 1),
    };
    const bg_style: svg.Style = .{ .fill_active = true, .fill_hsl = .{ 0, 0, 95 } };
    const line_style: svg.Style = .{ .stroke_width = 2, .stroke_hsl = .{ 90, 60, 40 } };
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
        var test_canvas = try svg.Canvas.init(test_alloc, min_pt, max_pt, bg_style);
        defer test_canvas.deinit(test_alloc);
        try test_canvas.addPolyline(test_alloc, &pts, line_style);
        var buf: [128]u8 = undefined;
        if (write_line_length) {
            const length_str = try std.fmt.bufPrint(&buf, "length = {d:.1}\n", .{curve_lenth});
            const text_loc = min_pt + calc.scaledVec(0.5, max_pt - min_pt);
            try test_canvas.addText(test_alloc, text_loc, length_str, 20, .{ 0, 0, 0 });
        }
        const fpath = try std.fmt.bufPrint(&buf, "{s}/{any}.html", .{ test_dir, Indexer });
        try test_canvas.writeToFile(testing.io, test_alloc, fpath, true);
        // operator should inspect the output: expect("looks good to me")
    }
}
