//! Helper module for generating random data and recorded statistics.
const std = @import("std");
const calc = @import("calc.zig");
const volume = @import("volume.zig");
const math = std.math;
const Allocator = std.mem.Allocator;
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

pub fn elapsedMs(t1: std.Io.Timestamp, t2: std.Io.Timestamp) f64 {
    return elapsedNs(t1, t2) / @as(f64, @floatCast(std.time.ns_per_ms));
}

pub fn elapsedNs(t1: std.Io.Timestamp, t2: std.Io.Timestamp) f64 {
    return @floatFromInt(std.Io.Timestamp.durationTo(t1, t2).toNanoseconds());
}

pub fn elapsedUs(t1: std.Io.Timestamp, t2: std.Io.Timestamp) f64 {
    return elapsedNs(t1, t2) / @as(f64, @floatCast(std.time.ns_per_us));
}

/// Stores several columns of same-typed data and provides helpers for computing stats + printing.
pub fn DataTable(
    comptime T: type,
    comptime num_cols: u8,
    comptime left_fmt: []const u8,
    comptime headers: [num_cols][]const u8,
    comptime formats: [num_cols][]const u8,
) type {
    const max_col_width = 32;

    return struct {
        column_data: [num_cols]std.ArrayList(T) = undefined,
        next_row: usize = 0,
        const Self = @This();

        pub fn init(allocator: Allocator, capacity: usize) !Self {
            std.debug.assert(capacity > 0);
            var cols: [num_cols]std.ArrayList(T) = undefined;
            var cols_created: usize = 0;
            errdefer for (0..cols_created) |i| cols[i].deinit(allocator);
            for (0..num_cols) |i| {
                cols[i] = try std.ArrayList(T).initCapacity(allocator, capacity);
                cols_created += 1;
            }
            return .{ .column_data = cols };
        }

        pub fn deinit(self: *Self, allocator: Allocator) void {
            for (0..num_cols) |i| self.column_data[i].deinit(allocator);
        }

        /// Adds a row to the table, overwriting the oldest one if at capacity.
        pub fn addRow(self: *Self, vals: [num_cols]T) void {
            if (self.column_data[0].items.len < self.column_data[0].capacity) {
                for (0..num_cols) |j| self.column_data[j].appendAssumeCapacity(vals[j]);
                self.next_row = self.column_data[0].items.len % self.column_data[0].capacity;
            } else {
                for (0..num_cols) |j| self.column_data[j].items[self.next_row] = vals[j];
                self.next_row = (self.next_row + 1) % self.column_data[0].items.len;
            }
        }

        /// Clears columns' contents without releasing their backing memory.
        pub fn clear(self: *Self) void {
            for (0..num_cols) |j| self.column_data[j].clearRetainingCapacity();
            self.next_row = 0;
        }

        /// Gets percentile stats for each column.
        /// Doesn't interpolate between indexes: inaccurate at low sample sizes.
        pub fn getPercentileStats(self: Self, scratch: []T, pct: u8) ?[num_cols]T {
            const len = self.column_data[0].items.len;
            if (len == 0) return null;
            std.debug.assert(scratch.len >= len);
            std.debug.assert(pct <= 100);

            var col_stats: [num_cols]T = undefined;
            for (0..num_cols) |j| {
                const items = self.column_data[j].items;
                const idx = @min(items.len - 1, @as(usize, pct) * items.len / 100);
                @memcpy(scratch[0..items.len], items);
                std.sort.pdq(T, scratch[0..items.len], {}, std.sort.asc(T));
                col_stats[j] = scratch[idx];
            }
            return col_stats;
        }

        /// Appends a header string to the array list.
        pub fn appendHeader(
            _: Self,
            allocator: Allocator,
            str_list: *std.ArrayList(u8),
            left_header: []const u8,
        ) !void {
            var left_buf: [max_col_width]u8 = undefined;
            const left_str = try std.fmt.bufPrint(&left_buf, left_fmt, .{left_header});
            try str_list.appendSlice(allocator, left_str);
            for (0..num_cols) |j| {
                try str_list.appendSlice(allocator, headers[j]);
                try str_list.append(allocator, '|');
            }
            try str_list.append(allocator, '\n');
        }

        /// Appends a row string to the array list.
        pub fn appendStatsRow(
            self: Self,
            allocator: Allocator,
            str_list: *std.ArrayList(u8),
            row_title: []const u8,
            pct: u8,
        ) !void {
            const scratch = try allocator.alloc(T, self.column_data[0].capacity);
            defer allocator.free(scratch);

            const col_stats = self.getPercentileStats(scratch, pct) orelse return error.NoValues;
            var left_buf: [max_col_width]u8 = undefined;
            const left_str = try std.fmt.bufPrint(&left_buf, left_fmt, .{row_title});
            try str_list.appendSlice(allocator, left_str);
            var fmt_buf: [max_col_width]u8 = undefined;
            inline for (0..num_cols) |i| {
                const cell_val = col_stats[i];
                const stat_str = try std.fmt.bufPrint(&fmt_buf, formats[i], .{cell_val});
                try str_list.appendSlice(allocator, stat_str);
                try str_list.append(allocator, '|');
            }
            try str_list.append(allocator, '\n');
        }

        /// Builds a multi-line string of column stats: one row for each percentile.
        /// Caller owns the returned slice.
        pub fn getStatsTable(
            self: Self,
            allocator: Allocator,
            left_header: []const u8, // header for the left-most column
            percentiles: []const u8, // numeric values, e.g. {50, 95}
        ) ![]u8 {
            const reserve_size = max_col_width * @as(usize, num_cols) * (1 + percentiles.len);
            var str_list = try std.ArrayList(u8).initCapacity(allocator, reserve_size);
            errdefer str_list.deinit(allocator);
            try self.appendHeader(allocator, &str_list, left_header);
            for (percentiles) |pct| {
                var buf: [8]u8 = undefined;
                const title = try std.fmt.bufPrint(&buf, "p{d:2}", .{pct});
                try self.appendStatsRow(allocator, &str_list, title, pct);
            }
            try str_list.appendSlice(allocator, "\n");
            return str_list.toOwnedSlice(allocator);
        }
    };
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

/// Container for test volumes (randomly-generated or loaded from a file).
/// TODO: add test lines!
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
    pub fn initCsv(allocator: Allocator, io: std.Io, filepath: []const u8) !Self {
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

    pub fn deinit(self: *Self, allocator: Allocator) void {
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
