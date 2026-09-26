//! Definitions of core volume types and related functions.
const std = @import("std");
const calc = @import("calc.zig");
const math = std.math;
pub const Vec2f = @Vector(2, f32);
const ProbDensityFunc = calc.ProbDensityFunc;

/// Any box where max[0] < min[0] is considered empty; choose extreme values to make this obvious.
pub const empty_box = Box2f{
    .min = @splat(math.floatMax(f32)),
    .max = @splat(-math.floatMax(f32)),
};

/// A circular region in the 2D plane.
pub const Ball2f = struct {
    centre: [2]f32,
    radius: f32,
    const Self = @This();

    pub fn getBoundingBox(self: Self) Box2f {
        if (self.isEmpty()) return empty_box;
        const disp: Vec2f = @splat(self.radius);
        const c: Vec2f = self.centre;
        return .{ .min = c - disp, .max = c + disp };
    }

    pub fn getCentre(self: Self) Vec2f {
        return self.centre;
    }

    pub fn getExpanded(self: Self, translation: Vec2f) Ball2f {
        return .{
            .centre = self.centre + calc.scaledVec(0.5, translation),
            .radius = self.radius + 0.5 * calc.norm(translation),
        };
    }

    pub fn getScaled(self: Self, factor: f32) Ball2f {
        return .{
            .centre = self.centre,
            .radius = @abs(factor) * self.radius,
        };
    }

    pub fn isEmpty(self: Self) bool {
        return self.radius <= 0;
    }
};

/// An axis-aligned rectangular region in the 2D plane.
pub const Box2f = struct {
    min: [2]f32,
    max: [2]f32,
    const Self = @This();

    pub fn getBoundingBox(self: Self) Box2f {
        return self;
    }

    pub fn getCentre(self: Self) Vec2f {
        const vec_sum = @as(Vec2f, self.min) + @as(Vec2f, self.max);
        return calc.scaledVec(0.5, vec_sum);
    }

    pub fn getExpanded(self: Self, translation: Vec2f) Box2f {
        const t_min = @as(Vec2f, self.min) + translation;
        const t_max = @as(Vec2f, self.max) + translation;
        return .{
            .min = @min(self.min, t_min),
            .max = @max(self.max, t_max),
        };
    }

    pub fn getScaled(self: Self, factor: f32) Box2f {
        const c = self.getCentre();
        const min_vec = @as(Vec2f, self.min);
        const max_vec = @as(Vec2f, self.max);
        const h = calc.scaledVec(0.5 * factor, max_vec - min_vec); // note the 0.5
        return .{ .min = c - h, .max = c + h };
    }

    pub fn isEmpty(self: Self) bool {
        return self.max[0] < self.min[0];
    }
};

/// An oriented bounding box.
pub const OrientedBox2f = struct {
    axis: [2]f32, // unit vector along the box's local x-axis
    centre: [2]f32,
    half_extents: [2]f32,
    const Self = @This();

    pub fn getBoundingBox(self: Self) Box2f {
        if (self.isEmpty()) return empty_box;
        const hx = self.half_extents[0];
        const hy = self.half_extents[1];
        const ax = @abs(@as(Vec2f, self.axis));
        const ay = @abs(Vec2f{ -self.axis[1], self.axis[0] });
        const extent = calc.scaledVec(hx, ax) + calc.scaledVec(hy, @abs(ay));
        return .{ .min = self.centre - extent, .max = self.centre + extent };
    }

    pub fn getCentre(self: Self) Vec2f {
        return self.centre;
    }

    pub fn getCorners(self: Self) [4]Vec2f {
        const ay = Vec2f{ -self.axis[1], self.axis[0] };
        const ex = calc.scaledVec(self.half_extents[0], @as(Vec2f, self.axis));
        const ey = calc.scaledVec(self.half_extents[1], ay);
        return .{
            self.centre + ex + ey,
            self.centre - ex + ey,
            self.centre - ex - ey,
            self.centre + ex - ey,
        };
    }

    pub fn getExpanded(self: Self, translation: Vec2f) OrientedBox2f {
        const axis_y = Vec2f{ -self.axis[1], self.axis[0] };
        const half_trans = calc.scaledVec(0.5, translation);
        const he_growth = Vec2f{
            @abs(calc.dot(self.axis, half_trans)),
            @abs(calc.dot(axis_y, half_trans)),
        };
        return .{
            .axis = self.axis,
            .centre = self.centre + calc.scaledVec(0.5, translation),
            .half_extents = self.half_extents + he_growth,
        };
    }

    pub fn getScaled(self: Self, factor: f32) OrientedBox2f {
        return .{
            .centre = self.centre,
            .half_extents = calc.scaledVec(@abs(factor), self.half_extents),
            .axis = self.axis,
        };
    }

    pub fn isEmpty(self: Self) bool {
        return self.half_extents[0] <= 0 or self.half_extents[1] <= 0;
    }
};

/// A zero-width line segment: query only, can't be stored in a tree.
pub const Line2f = struct {
    start: [2]f32,
    end: [2]f32,
    const Self = @This();

    pub fn getBoundingBox(self: Self) Box2f {
        return .{ .min = @min(self.start, self.end), .max = @max(self.start, self.end) };
    }
};

/// Returns true if the two volumes overlap.
pub fn checkVolumesOverlap(a: anytype, b: anytype) bool {
    return switch (@TypeOf(a)) {
        Ball2f => switch (@TypeOf(b)) {
            Ball2f => checkOverlapBallBall(a, b),
            Box2f => checkOverlapBallBox(a, b),
            OrientedBox2f => checkOverlapOrientedBoxBall(b, a),
            Line2f => checkOverlapLineBall(b, a),
            else => unreachable,
        },
        Box2f => switch (@TypeOf(b)) {
            Ball2f => checkOverlapBallBox(b, a),
            Box2f => checkOverlapBoxBox(a, b),
            OrientedBox2f => checkOverlapOrientedBoxBox(b, a),
            Line2f => checkOverlapLineBox(b, a),
            else => unreachable,
        },
        OrientedBox2f => switch (@TypeOf(b)) {
            Ball2f => checkOverlapOrientedBoxBall(a, b),
            Box2f => checkOverlapOrientedBoxBox(a, b),
            OrientedBox2f => checkOverlapOrientedBoxOrientedBox(a, b),
            Line2f => checkOverlapLineOrientedBox(b, a),
            else => unreachable,
        },
        Line2f => switch (@TypeOf(b)) {
            Ball2f => checkOverlapLineBall(a, b),
            Box2f => checkOverlapLineBox(a, b),
            OrientedBox2f => checkOverlapLineOrientedBox(a, b),
            else => unreachable,
        },
        else => unreachable, // overlap check has not been implemented for this volume
    };
}

/// Returns a box that covers both a and b.
pub fn getBoundingBox(a: anytype, b: anytype) Box2f {
    const box_a = a.getBoundingBox();
    const box_b = b.getBoundingBox();
    return .{
        .min = @min(@as(Vec2f, box_a.min), @as(Vec2f, box_b.min)),
        .max = @max(@as(Vec2f, box_a.max), @as(Vec2f, box_b.max)),
    };
}

fn checkOverlapBallBall(a: Ball2f, b: Ball2f) bool {
    const vec_diff = @as(Vec2f, a.centre) - @as(Vec2f, b.centre);
    const r_sum = a.radius + b.radius;
    return calc.squaredSum(vec_diff) < r_sum * r_sum;
}

fn checkOverlapBoxBox(a: Box2f, b: Box2f) bool {
    const lo = @shuffle(f32, a.min, b.min, [4]i32{ 0, 1, -1, -2 }); // {a.min, b.min}
    const hi = @shuffle(f32, b.max, a.max, [4]i32{ 0, 1, -1, -2 }); // {b.max, a.max}
    return @reduce(.And, lo < hi);
}

fn checkOverlapBallBox(a: Ball2f, b: Box2f) bool {
    const d_squared = calc.pointBoxDistSquared(a.centre, b.min, b.max);
    return d_squared < a.radius * a.radius;
}

fn checkOverlapLineBall(line: Line2f, ball: Ball2f) bool {
    const d_squared = calc.pointSegDistSquared(ball.centre, line.start, line.end);
    return d_squared < ball.radius * ball.radius;
}

fn checkOverlapLineBox(line: Line2f, box: Box2f) bool {
    return segmentIntersectsBox(line.start, line.end, box.min, box.max);
}

fn checkOverlapLineOrientedBox(line: Line2f, obb: OrientedBox2f) bool {
    const loc_start = calc.transformToFrame(line.start, obb.centre, obb.axis);
    const loc_end = calc.transformToFrame(line.end, obb.centre, obb.axis);
    const pos_he: Vec2f = obb.half_extents;
    const neg_he: Vec2f = -pos_he;
    return segmentIntersectsBox(loc_start, loc_end, neg_he, pos_he);
}

fn checkOverlapOrientedBoxBall(obb: OrientedBox2f, ball: Ball2f) bool {
    const local = calc.transformToFrame(ball.centre, obb.centre, obb.axis);
    const pos_he: Vec2f = obb.half_extents;
    const neg_he: Vec2f = -pos_he;
    const d_squared = calc.pointBoxDistSquared(local, neg_he, pos_he);
    return d_squared < ball.radius * ball.radius;
}

fn checkOverlapOrientedBoxBox(obb: OrientedBox2f, box: Box2f) bool {
    const box_half = calc.scaledVec(0.5, @as(Vec2f, box.max) - @as(Vec2f, box.min));
    return checkOverlapOrientedBoxes(
        obb.centre,
        obb.half_extents,
        obb.axis,
        box.getCentre(),
        box_half,
        [2]f32{ 1, 0 },
    );
}

fn checkOverlapOrientedBoxOrientedBox(a: OrientedBox2f, b: OrientedBox2f) bool {
    return checkOverlapOrientedBoxes(
        a.centre,
        a.half_extents,
        a.axis,
        b.centre,
        b.half_extents,
        b.axis,
    );
}

fn checkOverlapOrientedBoxes(
    c1: Vec2f,
    he1: Vec2f,
    ax1: Vec2f,
    c2: Vec2f,
    he2: Vec2f,
    ax2: Vec2f,
) bool {
    const ay1 = Vec2f{ -ax1[1], ax1[0] };
    const ay2 = Vec2f{ -ax2[1], ax2[0] };
    const d = c2 - c1;
    const axes = [4]Vec2f{ ax1, ay1, ax2, ay2 };
    for (axes) |axis| {
        const dist = @abs(calc.dotProduct(d, axis));
        const x_radius_1 = he1[0] * @abs(calc.dotProduct(ax1, axis));
        const y_radius_1 = he1[1] * @abs(calc.dotProduct(ay1, axis));
        const x_radius_2 = he2[0] * @abs(calc.dotProduct(ax2, axis));
        const y_radius_2 = he2[1] * @abs(calc.dotProduct(ay2, axis));
        const radius_1 = x_radius_1 + y_radius_1;
        const radius_2 = x_radius_2 + y_radius_2;
        if (dist >= radius_1 + radius_2) return false;
    }
    return true;
}

/// Adapted from Real-Time Collision Detection by Christer Ericson (Ch. 5.3.3).
fn segmentIntersectsBox(start: Vec2f, end: Vec2f, box_min: Vec2f, box_max: Vec2f) bool {
    const d = end - start;
    var t_min: f32 = 0;
    var t_max: f32 = 1;
    inline for (0..2) |i| {
        if (d[i] == 0) {
            if (start[i] < box_min[i] or start[i] > box_max[i]) return false;
        } else {
            const inv_d = 1.0 / d[i];
            var t1 = (box_min[i] - start[i]) * inv_d;
            var t2 = (box_max[i] - start[i]) * inv_d;
            if (t1 > t2) {
                const tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            t_min = @max(t_min, t1);
            t_max = @min(t_max, t2);
            if (t_min > t_max) return false;
        }
    }
    return true;
}

const testing = std.testing;

test "balls overlap" {
    const a = Ball2f{ .centre = .{ 0, 0 }, .radius = 1.0 };
    const b1 = Ball2f{ .centre = .{ 0.5, 0.5 }, .radius = 0.1 };
    const b2 = Ball2f{ .centre = .{ 1.5, 0.0 }, .radius = 0.6 };
    const b3 = Ball2f{ .centre = .{ 1.0, 1.0 }, .radius = 0.4 };
    const b4 = Ball2f{ .centre = .{ 1.5, 0.0 }, .radius = 0.5 };
    const check_1 = checkOverlapBallBall(a, b1);
    try testing.expectEqual(true, check_1);
    const check_2 = checkOverlapBallBall(a, b2);
    try testing.expectEqual(true, check_2);
    const check_3 = checkOverlapBallBall(a, b3);
    try testing.expectEqual(false, check_3);
    const check_4 = checkOverlapBallBall(a, b4);
    try testing.expectEqual(false, check_4);
}

test "boxes overlap" {
    const a = Box2f{ .min = .{ 0.0, 0.0 }, .max = .{ 1.0, 1.0 } };
    const b1 = Box2f{ .min = .{ 0.25, 0.25 }, .max = .{ 1.25, 1.25 } };
    const b2 = Box2f{ .min = .{ 0.5, 0.5 }, .max = .{ 0.75, 0.75 } };
    const b3 = Box2f{ .min = .{ 0.75, 0.75 }, .max = .{ 1.5, 1.5 } };
    const b4 = Box2f{ .min = .{ 1.5, 1.5 }, .max = .{ 1.75, 1.75 } };
    const check_1 = checkOverlapBoxBox(a, b1);
    const check_2 = checkOverlapBoxBox(a, b2);
    const check_3 = checkOverlapBoxBox(a, b3);
    const check_4 = checkOverlapBoxBox(a, b4);
    try testing.expectEqual(true, check_1);
    try testing.expectEqual(true, check_2);
    try testing.expectEqual(true, check_3);
    try testing.expectEqual(false, check_4);
}

test "ball-box overlap" {
    const a = Ball2f{ .centre = .{ 0.0, 0.0 }, .radius = 3 };
    const b1 = Box2f{ .min = .{ -1.0, -0.5 }, .max = .{ 1.0, 0.5 } };
    const b2 = Box2f{ .min = .{ -3.0, 4.5 }, .max = .{ 3.0, 5.5 } };
    const b3 = Box2f{ .min = .{ -0.5, -8.0 }, .max = .{ 0.5, -2.0 } };
    const b4 = Box2f{ .min = .{ -4.5, -4.5 }, .max = .{ -3.5, -3.5 } };
    const b5 = Box2f{ .min = .{ 1.5, 1.0 }, .max = .{ 2.5, 7.0 } };
    const check_1 = checkVolumesOverlap(a, b1);
    const check_2 = checkVolumesOverlap(a, b2);
    const check_3 = checkVolumesOverlap(a, b3);
    const check_4 = checkVolumesOverlap(a, b4);
    const check_5 = checkVolumesOverlap(a, b5);
    try testing.expectEqual(true, check_1);
    try testing.expectEqual(false, check_2);
    try testing.expectEqual(true, check_3);
    try testing.expectEqual(false, check_4);
    try testing.expectEqual(true, check_5);
}

test "oriented box - ball overlap" {
    const obb = OrientedBox2f{
        .centre = .{ 0, 0 },
        .half_extents = .{ 2, 1 },
        .axis = .{ 0, 1 },
    };
    const b1 = Ball2f{ .centre = .{ 0.5, 1.5 }, .radius = 0.6 };
    const b2 = Ball2f{ .centre = .{ 5, 5 }, .radius = 0.5 };
    const b3 = Ball2f{ .centre = .{ 1.5, 0 }, .radius = 0.6 };
    const b4 = Ball2f{ .centre = .{ 1.5, 0 }, .radius = 0.4 };
    try testing.expectEqual(true, checkVolumesOverlap(obb, b1));
    try testing.expectEqual(false, checkVolumesOverlap(obb, b2));
    try testing.expectEqual(true, checkVolumesOverlap(obb, b3));
    try testing.expectEqual(false, checkVolumesOverlap(obb, b4));
}

test "oriented box - box overlap" {
    const obb = OrientedBox2f{
        .centre = .{ 0, 0 },
        .half_extents = .{ 3, 1 },
        .axis = .{ 0, 1 },
    };
    const b1 = Box2f{ .min = .{ -0.5, -0.5 }, .max = .{ 0.5, 0.5 } };
    const b2 = Box2f{ .min = .{ 2, 2 }, .max = .{ 3, 3 } };
    const b3 = Box2f{ .min = .{ -10, -10 }, .max = .{ -9, -9 } };
    const b4 = Box2f{ .min = .{ 0.8, 2.8 }, .max = .{ 1.5, 3.5 } };
    try testing.expectEqual(true, checkVolumesOverlap(obb, b1));
    try testing.expectEqual(false, checkVolumesOverlap(obb, b2));
    try testing.expectEqual(false, checkVolumesOverlap(obb, b3));
    try testing.expectEqual(true, checkVolumesOverlap(obb, b4));
}

test "oriented box - oriented box overlap" {
    const a = OrientedBox2f{
        .centre = .{ 0, 0 },
        .half_extents = .{ 1, 1 },
        .axis = .{ math.sqrt1_2, math.sqrt1_2 },
    };
    const b1 = OrientedBox2f{
        .centre = .{ 0, 0 },
        .half_extents = .{ 0.2, 0.2 },
        .axis = .{ 1, 0 },
    };
    const b2 = OrientedBox2f{
        .centre = .{ 1.15, 1.15 },
        .half_extents = .{ 0.15, 0.15 },
        .axis = .{ 1, 0 },
    };
    const b3 = OrientedBox2f{
        .centre = .{ 5, 5 },
        .half_extents = .{ 1, 1 },
        .axis = .{ 0, 1 },
    };
    try testing.expectEqual(true, checkVolumesOverlap(a, b1));
    try testing.expectEqual(false, checkVolumesOverlap(a, b2));
    try testing.expectEqual(false, checkVolumesOverlap(a, b3));
    try testing.expectEqual(true, checkVolumesOverlap(b1, a));
}

test "line-ball overlap" {
    const line = Line2f{ .start = .{ 0, 0 }, .end = .{ 4, 0 } };
    const b1 = Ball2f{ .centre = .{ 2, 0.3 }, .radius = 0.4 };
    const b2 = Ball2f{ .centre = .{ 2, 3.0 }, .radius = 0.4 };
    const b3 = Ball2f{ .centre = .{ -0.1, 0 }, .radius = 0.15 };
    const b4 = Ball2f{ .centre = .{ 4.1, 0 }, .radius = 0.15 };
    const b5 = Ball2f{ .centre = .{ 5, 0 }, .radius = 0.5 };
    try testing.expectEqual(true, checkVolumesOverlap(line, b1));
    try testing.expectEqual(false, checkVolumesOverlap(line, b2));
    try testing.expectEqual(true, checkVolumesOverlap(line, b3));
    try testing.expectEqual(true, checkVolumesOverlap(line, b4));
    try testing.expectEqual(false, checkVolumesOverlap(line, b5));
    try testing.expectEqual(true, checkVolumesOverlap(b1, line));
    try testing.expectEqual(false, checkVolumesOverlap(b5, line));
}

test "line - oriented box overlap" {
    const obb = OrientedBox2f{ .centre = .{ 0, 0 }, .half_extents = .{ 3, 1 }, .axis = .{ 0, 1 } };
    const through = Line2f{ .start = .{ -2, 0 }, .end = .{ 2, 0 } };
    const miss_far = Line2f{ .start = .{ 2, 2 }, .end = .{ 3, 3 } };
    const miss_rotated = Line2f{ .start = .{ 1.5, 0 }, .end = .{ 1.5, 5 } };
    const inside = Line2f{ .start = .{ 0.9, 2.9 }, .end = .{ 1.2, 3.2 } };
    try testing.expectEqual(true, checkVolumesOverlap(through, obb));
    try testing.expectEqual(false, checkVolumesOverlap(miss_far, obb));
    try testing.expectEqual(false, checkVolumesOverlap(miss_rotated, obb));
    try testing.expectEqual(true, checkVolumesOverlap(inside, obb));
    try testing.expectEqual(true, checkVolumesOverlap(obb, through));
    try testing.expectEqual(false, checkVolumesOverlap(obb, miss_rotated));
}

test "line-box overlap" {
    const line = Line2f{ .start = .{ 0, 0 }, .end = .{ 4, 0 } };
    const b1 = Box2f{ .min = .{ 1, -0.2 }, .max = .{ 3, 0.2 } };
    const b2 = Box2f{ .min = .{ 1, 2 }, .max = .{ 3, 3 } };
    const b3 = Box2f{ .min = .{ -3, -3 }, .max = .{ -2, -2 } };
    const b4 = Box2f{ .min = .{ -0.5, -0.5 }, .max = .{ 4.5, 0.5 } };
    const b5 = Box2f{ .min = .{ 4, -1 }, .max = .{ 5, 1 } };
    try testing.expectEqual(true, checkVolumesOverlap(line, b1));
    try testing.expectEqual(false, checkVolumesOverlap(line, b2));
    try testing.expectEqual(false, checkVolumesOverlap(line, b3));
    try testing.expectEqual(true, checkVolumesOverlap(line, b4));
    try testing.expectEqual(true, checkVolumesOverlap(line, b5));
    try testing.expectEqual(true, checkVolumesOverlap(b1, line));
    try testing.expectEqual(false, checkVolumesOverlap(b2, line));
}

test "encompassing boxes" {
    const a = Box2f{ .min = .{ -0.139, -0.139 }, .max = .{ 0.139, 0.139 } };
    const b = Box2f{ .min = .{ -0.735, -0.2 }, .max = .{ 0.2, 0.735 } };
    const c = getBoundingBox(a, b);
    try testing.expectEqual(@min(@as(Vec2f, a.min), @as(Vec2f, b.min)), c.min);
    try testing.expectEqual(@max(@as(Vec2f, a.max), @as(Vec2f, b.max)), c.max);
}
