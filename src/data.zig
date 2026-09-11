//! This module is for general-purpose data structures used around the project.
const std = @import("std");

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

/// Stores a pair of values and provides helpers for sorting them.
pub fn Pair(A: type, B: type) type {
    return struct {
        a: A,
        b: B,
        const Self = @This();

        pub fn greaterThanA(_: void, x: Self, y: Self) bool {
            return x.a > y.a;
        }

        pub fn lessThanA(_: void, x: Self, y: Self) bool {
            return x.a < y.a;
        }

        pub fn greaterThanB(_: void, x: Self, y: Self) bool {
            return x.a > y.a;
        }

        pub fn lessThanB(_: void, x: Self, y: Self) bool {
            return x.a < y.a;
        }
    };
}
