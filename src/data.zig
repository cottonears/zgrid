//! This module is for general-purpose data structures used around the project.
const std = @import("std");
const AtomicUsize = std.atomic.Value(usize);
const Thread = std.Thread;
const cache_line = std.atomic.cache_line;

/// Wraps a slice of caller-owned memory, tracking how much of it is filled.
/// Holds mutable cursor state: always pass/store by pointer (`*BoundedList(T)`).
pub fn BoundedList(comptime T: type) type {
    return struct {
        index: usize = 0,
        items: []T = undefined,
        const Self = @This();

        /// Inits an empty list backed by a slice of caller-owned memory.
        pub fn init(slice: []T) Self {
            return .{ .items = slice };
        }

        /// Appends an item; returns BufferCapacityExceeded if at capacity.
        pub fn add(self: *Self, item: T) !void {
            if (self.index >= self.items.len) return error.BufferCapacityExceeded;
            self.items[self.index] = item;
            self.index += 1;
        }

        /// Empties the list without releasing its backing memory.
        pub fn clear(self: *Self) void {
            self.index = 0;
        }

        /// Gets a slice containing the current items.
        pub fn getItems(self: *const Self) []T {
            return self.items[0..self.index];
        }

        pub fn sortAsc(self: *Self) void {
            std.sort.pdq(T, self.items[0..self.index], {}, asc);
        }

        pub fn sortDesc(self: *Self) void {
            std.sort.pdq(T, self.items[0..self.index], {}, desc);
        }

        fn asc(_: void, a: T, b: T) bool {
            return a < b;
        }
        fn desc(_: void, a: T, b: T) bool {
            return a > b;
        }
    };
}

pub fn DataTable(
    comptime T: type,
    comptime num_cols: u8,
    comptime headers: [num_cols][]const u8,
    comptime formats: [num_cols][]const u8,
) type {
    return struct {
        column_data: [num_cols]BoundedList(T) = undefined,
        num_rows: usize = 0,
        const Self = @This();

        pub fn init(allocator: std.mem.Allocator, capacity: usize) !Self {
            var cols: [num_cols]BoundedList(T) = undefined;
            var cols_created: usize = 0;
            errdefer for (0..cols_created) |i| allocator.free(cols[i].items);
            for (0..num_cols) |i| {
                const col_slice = try allocator.alloc(T, capacity);
                cols[i] = BoundedList(T).init(col_slice);
                cols_created += 1;
            }
            return .{ .column_data = cols };
        }

        pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
            for (0..num_cols) |i| allocator.free(self.column_data[i].items);
        }

        pub fn addRow(self: *Self, vals: [num_cols]T) !void {
            for (0..num_cols) |j| try self.column_data[j].add(vals[j]);
            self.num_rows += 1;
        }

        /// Clears columns' contents.
        pub fn clear(self: *Self) void {
            for (0..num_cols) |j| self.column_data[j].clear();
            self.num_rows = 0;
        }

        /// Sorts column data and gets range + IQR stats for each: { min, q1, q2, q3, max }.
        /// Doesn't interpolate between indexes: inacurate for a low sample sizes.
        pub fn computeStats(self: *Self) ?[num_cols][5]T {
            if (self.column_data[0].index == 0) return null;
            var col_stats: [num_cols][5]T = undefined;
            for (0..num_cols) |j| {
                self.column_data[j].sortAsc();
                const items = self.column_data[j].getItems();
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

pub const Range = struct { start: usize, end: usize };

/// Padded so it lives in its own cache line (and doesn't evict useful data when incremented).
const AtomicCounter = struct {
    value: AtomicUsize align(std.atomic.cache_line) = AtomicUsize.init(0),
    _padding: [cache_line - @sizeOf(AtomicUsize)]u8 = undefined,
};

/// Simple index-based partitioner to support lock-free multithreading
pub const AtomicRangeIter = struct {
    r: AtomicCounter = .{}, // index of the next range
    // r: AtomicUsize = AtomicUsize.init(0),
    num_ranges: usize, // number of ranges
    quotient: usize, // base length given to all ranges
    remainder: usize, // remaining length split among early ranges
    start: usize, // start index
    const Self = @This();

    /// Creates a range iterator that subdivides the range from [start, end).
    pub fn init(start: usize, end: usize, num_partitions: usize) !Self {
        if (start > end) return error.InvalidIndexes;
        if (num_partitions < 1) return error.InvalidNumberPartitions;
        if (start == end) {
            return .{ .num_ranges = 1, .quotient = 0, .remainder = 0, .start = start };
        } else {
            const n = @min(num_partitions, end - start);
            const total_len = end - start;
            return .{
                .num_ranges = n,
                .quotient = total_len / n,
                .remainder = total_len % n,
                .start = start,
            };
        }
    }

    /// Gets the next range, or null if all ranges were already retrieved.
    pub fn next(self: *Self) ?Range {
        const i = self.r.value.fetchAdd(1, .monotonic);
        // const i = self.r.fetchAdd(1, .monotonic);
        if (i >= self.num_ranges) return null;
        // previous i ranges will have had remainder distributed among them
        const rem_accumulated = @min(i, self.remainder);
        const range_start = self.start + rem_accumulated + i * self.quotient;
        const len = self.quotient + @intFromBool(i < self.remainder);
        return .{ .start = range_start, .end = range_start + len };
    }
};

const testing = std.testing;
const test_alloc = testing.allocator;

test "add to data table" {
    const col_headers: [3][]const u8 = .{ " count ", " pressure   ", " temperature " };
    const col_formats: [3][]const u8 = .{ " {d:>5.0} ", " {d:>6.1} kPa ", " {d:>9.2}°K " };
    var my_table = try DataTable(f64, 3, col_headers, col_formats).init(test_alloc, 100);
    defer my_table.deinit(test_alloc);
    const n = 42.0;
    for (0..100) |i| {
        const t = 260 + @as(f64, @floatFromInt(i));
        const p = 8.314 * 42.29 * t / 1000.0;
        try my_table.addRow(.{ n, p, t });
    }
    const table_str = try my_table.getStatsTable(test_alloc);
    defer test_alloc.free(table_str);
    // std.debug.print("{s}", .{table_str});
}

test "test range iterator" {
    const desired_parts = 13;
    const len_max = 1000;
    const num_tests = 1000;
    const start_max = 1000;
    var prng = std.Random.DefaultPrng.init(0);
    var random = prng.random();
    for (0..num_tests) |_| {
        const start = random.uintAtMost(usize, start_max);
        const end = start + random.uintAtMost(usize, len_max);
        var len_covered: usize = 0;
        var range_iter = try AtomicRangeIter.init(start, end, desired_parts);
        while (range_iter.next()) |p| {
            try testing.expect(p.start >= start);
            try testing.expect(p.end <= end);
            len_covered += p.end - p.start;
        }
        try testing.expectEqual(len_covered, end - start);
    }
}

test "check AtomicRangeIter layout" {
    std.debug.print("alignOf(AtomicRangeIter) = {}\n", .{@alignOf(AtomicRangeIter)});
    std.debug.print("sizeOf(AtomicRangeIter)  = {}\n", .{@sizeOf(AtomicRangeIter)});
    std.debug.print("alignOf(AtomicUsize)     = {}\n", .{@alignOf(AtomicUsize)});
    std.debug.print("cache line               = {}\n", .{std.atomic.cache_line});

    std.debug.print("offset r          = {}\n", .{@offsetOf(AtomicRangeIter, "r")});
    std.debug.print("offset num_ranges  = {}\n", .{@offsetOf(AtomicRangeIter, "num_ranges")});
    std.debug.print("offset quotient    = {}\n", .{@offsetOf(AtomicRangeIter, "quotient")});
    std.debug.print("offset remainder   = {}\n", .{@offsetOf(AtomicRangeIter, "remainder")});
    std.debug.print("offset start       = {}\n", .{@offsetOf(AtomicRangeIter, "start")});
}
