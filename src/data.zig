//! This module is for general-purpose data structures used around the project.
const std = @import("std");

/// Wraps a slice of caller-owned memory, tracking how much of it is filled.
/// Holds mutable cursor state: always pass/store by pointer (`*BoundedList(T)`).
pub fn BoundedList(comptime T: type) type {
    return struct {
        index: usize = 0,
        slice: []T = undefined,
        const Self = @This();

        /// Inits an empty list backed by a slice of caller-owned memory.
        pub fn init(buf: []T) Self {
            return .{ .slice = buf };
        }

        /// Appends an item; returns BufferCapacityExceeded if at capacity.
        pub fn add(self: *Self, item: T) !void {
            if (self.index >= self.slice.len) return error.BufferCapacityExceeded;
            self.slice[self.index] = item;
            self.index += 1;
        }

        pub fn addRange(self: *Self, items: []T) !void {
            if (self.index + items.len > self.slice.len) return error.BufferCapacityExceeded;
            @memcpy(self.slice[self.index..][0..items.len], items[0..]);
            self.index += items.len;
        }

        /// Empties the list without releasing its backing memory.
        pub fn clear(self: *Self) void {
            self.index = 0;
        }

        /// Gets a slice containing the current items.
        pub fn getItems(self: *const Self) []T {
            return self.slice[0..self.index];
        }

        // TODO: remove the below and add to data-table
        pub fn sortAsc(self: *Self) void {
            std.sort.pdq(T, self.slice[0..self.index], {}, asc);
        }

        pub fn sortDesc(self: *Self) void {
            std.sort.pdq(T, self.slice[0..self.index], {}, desc);
        }

        fn asc(_: void, a: T, b: T) bool {
            return a < b;
        }
        fn desc(_: void, a: T, b: T) bool {
            return a > b;
        }
    };
}
