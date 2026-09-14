const std = @import("std");
const Group = std.Io.Group;
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

// Wraps a caller-supplied buffer + atomic counter to support lightweight thread-safe writing.
pub fn SharedBuffer(comptime T: type) type {
    return struct {
        counter: AtomicCounter = .{}, // this can be incremented past items.len (overflow case)
        index: usize = 0, // indexes items (does not increment past length)
        items: []T,
        const Self = @This();

        /// Inits an empty list backed by a slice of caller-owned memory.
        pub fn init(buf: []T) Self {
            return .{ .items = buf };
        }

        /// Copies items from the provided slice to this buffer's storage: thread-safe.
        /// No items will be copied once buffer capacity is exceeded.
        pub fn appendSlice(self: *Self, slice: []T) void {
            if (slice.len == 0) return;
            const start = self.counter.value.fetchAdd(slice.len, .monotonic);
            // TODO: check the below carefully
            const end = @min(self.items.len, start + slice.len);
            if (start < end) {
                @memcpy(self.items[start..end], slice[0..(end - start)]);
                self.index += end - start;
            }
        }

        /// Empties the list without releasing its backing memory.
        pub fn clear(self: *Self) void {
            self.counter.value = AtomicUsize.init(0); // correct?
            self.index = 0;
        }

        /// Gets a slice containing the all stored items.
        /// Returns an error if the buffer's capacity was exceeded.
        pub fn getItems(self: *const Self) ![]T {
            const len = self.counter.value.load(.monotonic);
            if (len > self.index) return error.SharedBufferCapacityExceeded;
            return self.getItemsNoError();
        }

        /// Gets a slice containing the current items; never returns an error.
        /// Will exclude any items that couldn't be stored after the buffer reached capacity.
        pub fn getItemsNoError(self: *const Self) []T {
            return self.items[0..self.index];
        }
    };
}

/// Claims room in an output buffer by incrementing the atomic counter, writes results, then empties the list
pub fn copyToSharedBuffer(
    comptime T: type,
    res_list: *std.ArrayList(T),
    out_counter: *AtomicCounter,
    output: []T,
) void {
    const items = res_list.items;
    if (items.len == 0) return;
    const start = out_counter.value.fetchAdd(items.len, .monotonic);
    // TODO: struct version should write up until the end of the buffer (friendlier for callers who may want to partially recover).
    if (start + items.len <= output.len) @memcpy(output[start..][0..items.len], items);
    res_list.clearRetainingCapacity();
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

test "test write to shared buffer" {
    const SCRATCH_LEN = 64;
    const Work = struct {
        pub fn doWork(iter: *AtomicRangeIter, data: []usize, out: *SharedBuffer(usize)) void {
            var scratch_buf: [SCRATCH_LEN]usize = undefined;
            var scratch_list = std.ArrayList(usize).initBuffer(&scratch_buf);
            std.debug.print("scratch_list.capacity ={}\n", .{scratch_list.capacity});
            while (iter.next()) |r| {
                for (r.start..r.end) |i| {
                    const result = data[i] * data[i];
                    // if at capacity, copy scratch results to the output buffer + clear
                    if (scratch_list.items.len == scratch_list.capacity) {
                        out.appendSlice(scratch_list.items);
                        scratch_list.clearRetainingCapacity();
                        std.debug.print("scratch_list cleared\n", .{});
                    }
                    scratch_list.appendBounded(result) catch |e| {
                        std.debug.print("error in doWork: {}\n", .{e});
                    };
                }
            }
        }
    };

    var in_buf: [1024]usize = undefined;
    for (0..in_buf.len) |i| in_buf[i] = i;
    var out_buf: [1000]usize = undefined;
    var shared_buf = SharedBuffer(usize).init(&out_buf);
    var range_iter = AtomicRangeIter.init(0, in_buf.len, 8);
    var group: Group = .init;
    errdefer group.cancel(testing.io);
    const num_workers = 4;
    // NOTE: work split into 8 partitions (of length 128) shared among 4 workers with scratch_len = 64
    // This forces workers to clear their stack buffers twice per allocated partition
    for (0..num_workers) |_| {
        group.async(testing.io, Work.doWork, .{ &range_iter, &in_buf, &shared_buf });
    }
    try group.await(testing.io);
    try testing.expectError(error.SharedBufferCapacityExceeded, shared_buf.getItems());
    const out_slice = shared_buf.getItemsNoError();
    try testing.expectEqual(out_buf.len, out_slice.len);
    for (out_slice, 0..) |v, i| {
        const sqrt_v = std.math.sqrt(v);
        std.debug.print("{d}. {d} -> {d}\n", .{ i, sqrt_v, v });
    }
}
