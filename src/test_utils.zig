//! Helper module for generating random data and helping with tests
const std = @import("std");
const calc = @import("maths/calc.zig");
const prob = @import("maths/prob.zig");
const volume = @import("maths/volume.zig");
const math = std.math;
const Allocator = std.mem.Allocator;
const Timestamp = std.Io.Timestamp;
const Box2f = volume.Box2f;
const Ball2f = volume.Ball2f;
const Line2f = volume.Line2f;
const OrientedBox2f = volume.OrientedBox2f;
const ProbDensityFunc = prob.ProbDensityFunc;
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

pub fn elapsedMs(t1: Timestamp, t2: Timestamp) f64 {
    return elapsedNs(t1, t2) / std.time.ns_per_ms;
}

pub fn elapsedNs(t1: Timestamp, t2: Timestamp) f64 {
    return @floatFromInt(Timestamp.durationTo(t1, t2).toNanoseconds());
}

pub fn elapsedUs(t1: Timestamp, t2: Timestamp) f64 {
    return elapsedNs(t1, t2) / std.time.ns_per_us;
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
    if (num_cols == 0) @compileError("DataTable requires at least one column");

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

/// Container for test volumes (randomly-generated or loaded from a file).
pub const TestVolumes = struct {
    balls: std.ArrayList(Ball2f),
    boxes: std.ArrayList(Box2f),
    lines: std.ArrayList(Line2f),
    obbs: std.ArrayList(OrientedBox2f),
    const Self = @This();

    pub fn initRandom(
        allocator: std.mem.Allocator,
        random: std.Random,
        capacity: usize,
        size_dist: ProbDensityFunc,
        position_dist: ProbDensityFunc,
    ) !Self {
        var r_floats = try allocator.alloc(f32, capacity);
        defer allocator.free(r_floats);
        var r_vecs = try allocator.alloc(Vec2f, capacity);
        defer allocator.free(r_vecs);
        // generate random balls
        ProbDensityFunc.fillFloat(size_dist, random, r_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, r_vecs[0..]);
        var balls = try std.ArrayList(Ball2f).initCapacity(allocator, capacity);
        errdefer balls.deinit(allocator);
        for (0..capacity) |i| {
            const ball = Ball2f{ .centre = r_vecs[i], .radius = r_floats[i] };
            balls.appendAssumeCapacity(ball);
        }
        // generate random boxes
        ProbDensityFunc.fillFloat(size_dist, random, r_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, r_vecs[0..]);
        var boxes = try std.ArrayList(Box2f).initCapacity(allocator, capacity);
        errdefer boxes.deinit(allocator);
        for (0..capacity) |i| {
            const j = (i + capacity / 2) % capacity;
            const dim = Vec2f{ 2 * r_floats[i], 2 * r_floats[j] };
            const box = Box2f{ .min = r_vecs[i], .max = r_vecs[i] + dim };
            boxes.appendAssumeCapacity(box);
        }
        // generate random lines
        ProbDensityFunc.fillFloat(size_dist, random, r_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, r_vecs[0..]);
        var lines = try std.ArrayList(Line2f).initCapacity(allocator, capacity);
        errdefer lines.deinit(allocator);
        for (0..capacity) |i| {
            const length = r_floats[i];
            const start = r_vecs[i];
            const j = (i + capacity / 2) % capacity;
            const d = r_vecs[j] - start;
            const d_norm = calc.norm(d);
            const end = if (d_norm > 0.0)
                start + calc.scaledVec(length / d_norm, d)
            else
                start + calc.scaledVec(length, .{ 1, 0 });

            const line = Line2f{ .start = start, .end = end };
            lines.appendAssumeCapacity(line);
        }
        // generate random oriented boxes
        ProbDensityFunc.fillFloat(size_dist, random, r_floats[0..]);
        ProbDensityFunc.fillVec2f(position_dist, random, r_vecs[0..]);
        const tau_dist = ProbDensityFunc{ .uniform = .{ .min = 0, .max = math.tau } };
        var obbs = try std.ArrayList(OrientedBox2f).initCapacity(allocator, capacity);
        errdefer obbs.deinit(allocator);
        for (0..capacity) |i| {
            const j = (i + capacity / 2) % capacity;
            const angle = ProbDensityFunc.getFloat(tau_dist, random);
            const obb = OrientedBox2f{
                .centre = r_vecs[i],
                .half_extents = .{ r_floats[i], r_floats[j] },
                .axis = .{ @cos(angle), @sin(angle) },
            };
            obbs.appendAssumeCapacity(obb);
        }
        return .{
            .balls = balls,
            .boxes = boxes,
            .lines = lines,
            .obbs = obbs,
        };
    }

    pub fn deinit(self: *Self, allocator: Allocator) void {
        self.balls.deinit(allocator);
        self.boxes.deinit(allocator);
        self.lines.deinit(allocator);
        self.obbs.deinit(allocator);
    }

    pub fn getVolumes(self: *Self, comptime T: type) []T {
        return switch (T) {
            Ball2f => self.balls.items,
            Box2f => self.boxes.items,
            Line2f => self.lines.items,
            OrientedBox2f => self.obbs.items,
            else => @compileError("Unsupported volume type: " ++ @typeName(T)),
        };
    }

    pub fn totalVolumes(self: *const Self) usize {
        return self.balls.items.len +
            self.boxes.items.len +
            self.lines.items.len +
            self.obbs.items.len;
    }
};
