//! Helper module for generating random data.
const std = @import("std");
const calc = @import("calc.zig");
const volume = @import("volume.zig");
const math = std.math;
const Box2f = volume.Box2f;
const Ball2f = volume.Ball2f;
const OrientedBox2f = volume.OrientedBox2f;
const Vec2f = calc.Vec2f;

/// Gets a u64 based on system clock's measured nanoseconds - helpful in tests
pub fn getClockBasedRngSeed(io: std.Io) u64 {
    const now = std.Io.Clock.real.now(io);
    return @truncate(@abs(now.nanoseconds));
}

/// Use this in errdefer block to help reproduce an error that might be related to a random seed.
pub fn printErrorMessageForRandomSeed(seed: u64) void {
    std.debug.print("Error when testing with random data; seed = {d}\n", .{seed});
}

/// Defines a probability distribution used to generate f32 test data.
pub const ProbDensityFunc = union(enum) {
    uniform: struct { min: f32, max: f32 },
    normal: struct { mean: f32, stddev: f32 },

    /// Parses a pdf string of the form "U(min,max)" or "N(mean,stddev)".
    pub fn fromPdfString(str: []const u8) !ProbDensityFunc {
        const pdf_err = error.InvalidPdfArgs;
        if (str.len < 2 or str[1] != '(' or str[str.len - 1] != ')') return pdf_err;
        const params_str = str[2 .. str.len - 1];
        const comma_idx = std.mem.indexOfScalar(u8, params_str, ',');
        if (comma_idx == null) return error.InvalidPdfArgs;
        const a = try std.fmt.parseFloat(f32, params_str[0..comma_idx.?]);
        const b = try std.fmt.parseFloat(f32, params_str[comma_idx.? + 1 ..]);
        return switch (std.ascii.toUpper(str[0])) {
            'N' => if (a <= 0) pdf_err else .{ .normal = .{ .mean = a, .stddev = b } },
            'U' => if (a >= b) pdf_err else .{ .uniform = .{ .min = a, .max = b } },
            else => return error.UnrecognisedPdfType,
        };
    }

    pub fn getFloat(pdf: ProbDensityFunc, random: std.Random) f32 {
        return switch (pdf) {
            .normal => |n| n.mean + n.stddev * random.floatNorm(f32),
            .uniform => |u| u.min + (u.max - u.min) * random.float(f32),
        };
    }

    // Fills the provided slice with random floats according to the distribution.
    pub fn fillFloat(pdf: ProbDensityFunc, random: std.Random, slice: []f32) void {
        for (0..slice.len) |i| slice[i] = getFloat(pdf, random);
    }

    // Fills the provided slice with vectors with random coefficients (symmetric in both axes).
    pub fn fillVec2f(pdf: ProbDensityFunc, random: std.Random, vec_slice: []Vec2f) void {
        return fillFloat(pdf, random, @ptrCast(vec_slice));
    }
};

pub const TestVolumes = struct {
    balls: std.ArrayList(Ball2f),
    boxes: std.ArrayList(Box2f),
    oriented_boxes: std.ArrayList(OrientedBox2f),
    const Self = @This();

    pub fn initRandom(
        allocator: std.mem.Allocator,
        random: std.Random,
        capacity: usize,
        size_dist: ProbDensityFunc,
        position_dist: ProbDensityFunc,
    ) !Self {
        var random_floats = try allocator.alloc(f32, capacity);
        defer allocator.free(random_floats);
        var random_vecs = try allocator.alloc(Vec2f, capacity);
        defer allocator.free(random_vecs);
        // generate random balls
        ProbDensityFunc.fillFloat(size_dist, random, random_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, random_vecs[0..]);
        var rand_balls = try std.ArrayList(Ball2f).initCapacity(allocator, capacity);
        for (0..capacity) |i| {
            rand_balls.appendAssumeCapacity(.{
                .centre = random_vecs[i],
                .radius = random_floats[i],
            });
        }
        // generate random boxes
        ProbDensityFunc.fillFloat(size_dist, random, random_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, random_vecs[0..]);
        var rand_boxes = try std.ArrayList(Box2f).initCapacity(allocator, capacity);
        for (0..capacity) |i| {
            const j = (i + capacity / 2) % capacity;
            const dim = Vec2f{ 2 * random_floats[i], 2 * random_floats[j] };
            const box = Box2f{ .min = random_vecs[i], .max = random_vecs[i] + dim };
            rand_boxes.appendAssumeCapacity(box);
        }
        // generate random oriented boxes
        ProbDensityFunc.fillFloat(size_dist, random, random_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, random_vecs[0..]);
        const tau_dist = ProbDensityFunc{ .uniform = .{ .min = 0, .max = math.tau } };
        var rand_obbs = try std.ArrayList(OrientedBox2f).initCapacity(allocator, capacity);
        for (0..capacity) |i| {
            const j = (i + capacity / 2) % capacity;
            const angle = ProbDensityFunc.getFloat(tau_dist, random);
            rand_obbs.appendAssumeCapacity(.{
                .centre = random_vecs[i],
                .half_extents = .{ random_floats[i], random_floats[j] },
                .axis = .{ @cos(angle), @sin(angle) },
            });
        }
        return .{
            .balls = rand_balls,
            .boxes = rand_boxes,
            .oriented_boxes = rand_obbs,
        };
    }

    /// Loads test volumes from a csv file; rows must match below format:
    ///  - ball, centre_x, centre_y, radius
    ///  - box, min_x, min_y, max_x, max_y
    ///  - obb, centre_x, centre_y, half_extent_x, half_extent_y, axis_x, axis_y
    pub fn initCsv(allocator: std.mem.Allocator, io: std.Io, filepath: []const u8) !Self {
        const contents = try std.Io.Dir.cwd().readFileAlloc(io, filepath, allocator, .unlimited);
        defer allocator.free(contents);

        var ball_list: std.ArrayList(Ball2f) = .empty;
        errdefer ball_list.deinit(allocator);
        var box_list: std.ArrayList(Box2f) = .empty;
        errdefer box_list.deinit(allocator);
        var obb_list: std.ArrayList(OrientedBox2f) = .empty;
        errdefer obb_list.deinit(allocator);

        var lines = std.mem.tokenizeAny(u8, contents, "\r\n");
        while (lines.next()) |line| {
            const trimmed = std.mem.trim(u8, line, " \t");
            if (trimmed.len == 0) continue;
            var fields = std.mem.splitScalar(u8, trimmed, ',');
            const kind = fields.next() orelse return error.InvalidCsvRow;
            if (std.ascii.eqlIgnoreCase(kind, "ball")) {
                const cx = try parseCsvFloat(&fields);
                const cy = try parseCsvFloat(&fields);
                const r = try parseCsvFloat(&fields);
                try ball_list.append(allocator, .{ .centre = .{ cx, cy }, .radius = r });
            } else if (std.ascii.eqlIgnoreCase(kind, "box")) {
                const min_x = try parseCsvFloat(&fields);
                const min_y = try parseCsvFloat(&fields);
                const max_x = try parseCsvFloat(&fields);
                const max_y = try parseCsvFloat(&fields);
                try box_list.append(allocator, .{ .min = .{ min_x, min_y }, .max = .{ max_x, max_y } });
            } else if (std.ascii.eqlIgnoreCase(kind, "obb")) {
                const cx = try parseCsvFloat(&fields);
                const cy = try parseCsvFloat(&fields);
                const hx = try parseCsvFloat(&fields);
                const hy = try parseCsvFloat(&fields);
                const ax = try parseCsvFloat(&fields);
                const ay = try parseCsvFloat(&fields);
                try obb_list.append(allocator, .{
                    .centre = .{ cx, cy },
                    .half_extents = .{ hx, hy },
                    .axis = .{ ax, ay },
                });
            } else {
                return error.UnknownVolumeType;
            }
        }
        ball_list.shrinkAndFree(allocator, ball_list.items.len);
        box_list.shrinkAndFree(allocator, box_list.items.len);
        obb_list.shrinkAndFree(allocator, obb_list.items.len);

        return .{
            .balls = ball_list,
            .boxes = box_list,
            .oriented_boxes = obb_list,
        };
    }

    pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
        self.balls.deinit(allocator);
        self.boxes.deinit(allocator);
        self.oriented_boxes.deinit(allocator);
    }

    pub fn getVolumes(self: *Self, comptime T: type) []T {
        return switch (T) {
            Ball2f => self.balls.items,
            Box2f => self.boxes.items,
            OrientedBox2f => self.oriented_boxes.items,
            else => unreachable,
        };
    }

    fn parseCsvFloat(fields: *std.mem.SplitIterator(u8, .scalar)) !f32 {
        const field = fields.next() orelse return error.InvalidCsvRow;
        return std.fmt.parseFloat(f32, field);
    }
};
