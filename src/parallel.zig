const std = @import("std");
const data = @import("data.zig");
const AtomicUsize = std.atomic.Value(usize);
pub const Range = struct { start: usize, end: usize };

/// Defined in a separate struct and cache-aligned to prevent false sharing.
pub const AtomicCounter = struct {
    value: AtomicUsize align(std.atomic.cache_line) = AtomicUsize.init(0),
};

/// Simple index-based partitioner to support lock-free multithreading
pub const AtomicRangeIter = struct {
    r: AtomicCounter = .{},
    num_ranges: usize, // number of ranges
    quotient: usize, // base length given to all ranges
    remainder: usize, // remaining length split among early ranges
    start: usize, // start index
    const Self = @This();

    /// Creates a range iterator that subdivides the range from [start, end).
    pub fn init(start: usize, end: usize, num_partitions: usize) Self {
        std.debug.assert(start <= end);
        if (start == end) {
            return .{ .num_ranges = 1, .quotient = 0, .remainder = 0, .start = start };
        } else {
            const n = @max(1, @min(num_partitions, end - start));
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
        if (i >= self.num_ranges) return null;
        // previous i ranges will have had remainder distributed among them
        const rem_accumulated = @min(i, self.remainder);
        const range_start = self.start + rem_accumulated + i * self.quotient;
        const len = self.quotient + @intFromBool(i < self.remainder);
        return .{ .start = range_start, .end = range_start + len };
    }
};

// TODO: implement the below to handle the below functions + use it
/// Wraps a bound list + counter to support thread-safe writing to a shared buffer.
// pub fn SharedList(comptime T: type) type

/// Claims room in an output buffer by incrementing the atomic counter, writes results, then empties the list
pub fn copyToSharedBuffer(
    comptime T: type,
    res_list: *data.BoundedList(T),
    out_counter: *AtomicCounter,
    output: []T,
) void {
    const items = res_list.getItems();
    if (items.len == 0) return;
    const start = out_counter.value.fetchAdd(items.len, .monotonic);
    // TODO: struct version should write up until the end of the buffer (friendlier for callers who may want to partially recover).
    if (start + items.len <= output.len) @memcpy(output[start..][0..items.len], items);
    res_list.clear();
}

// TODO: struct version should provider friendlier version of the below, allowing all written results to be recovered
/// Gets the output slice, or an error if buffer capacity was exceeded
pub fn getOutputSlice(
    comptime T: type,
    out_counter: *const AtomicCounter,
    output: []T,
) ![]T {
    const len = out_counter.value.load(.monotonic);
    if (len > output.len) return error.BufferCapacityExceeded;
    return output[0..len];
}

const testing = std.testing;

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
        var range_iter = AtomicRangeIter.init(start, end, desired_parts);
        while (range_iter.next()) |p| {
            try testing.expect(p.start >= start);
            try testing.expect(p.end <= end);
            len_covered += p.end - p.start;
        }
        try testing.expectEqual(len_covered, end - start);
    }
}
