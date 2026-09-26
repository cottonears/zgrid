//! Defines the recursive space-filling curves use for indexing.
const std = @import("std");
const math = std.math;
pub const GridIndex = u8;

pub const Curve = enum {
    // TODO: add support for Hilbert curves! (will need to revise index.getCellBoundaryAtLevel)
    Morton2,
    Morton4,
    Morton8,
    Morton16,
    Morton32,
    Morton64,
    Morton128,
    Morton256,
    Spring4,
    Spring16,
    Spring64,
    Spring256,
    Zigzag4,
    Zigzag16,
    Zigzag64,
    Zigzag256,

    pub fn base(comptime curve: Curve) u8 {
        return switch (curve) {
            .Morton2, .Morton4, .Morton8, .Morton16, .Morton32, .Morton64, .Morton128, .Morton256 => 2,
            else => 4,
        };
    }

    pub fn degree(comptime curve: Curve) u4 {
        return switch (curve) {
            .Morton2 => 1,
            .Morton4 => 2,
            .Morton8 => 3,
            .Morton16 => 4,
            .Morton32 => 5,
            .Morton64 => 6,
            .Morton128 => 7,
            .Morton256 => 8,
            .Spring4 => 1,
            .Spring16 => 2,
            .Spring64 => 3,
            .Spring256 => 4,
            .Zigzag4 => 1,
            .Zigzag16 => 2,
            .Zigzag64 => 3,
            .Zigzag256 => 4,
        };
    }

    pub fn index(comptime curve: Curve) type {
        return switch (curve) {
            .Morton2 => u2,
            .Morton4 => u4,
            .Morton8 => u6,
            .Morton16 => u8,
            .Morton32 => u10,
            .Morton64 => u12,
            .Morton128 => u14,
            .Morton256 => u16,
            .Spring4 => u4,
            .Spring16 => u8,
            .Spring64 => u12,
            .Spring256 => u16,
            .Zigzag4 => u4,
            .Zigzag16 => u8,
            .Zigzag64 => u12,
            .Zigzag256 => u16,
        };
    }

    pub fn size(comptime curve: Curve) usize {
        return switch (curve) {
            .Morton2 => 2,
            .Morton4 => 4,
            .Morton8 => 8,
            .Morton16 => 16,
            .Morton32 => 32,
            .Morton64 => 64,
            .Morton128 => 128,
            .Morton256 => 256,
            .Spring4 => 4,
            .Spring16 => 16,
            .Spring64 => 64,
            .Spring256 => 256,
            .Zigzag4 => 4,
            .Zigzag16 => 16,
            .Zigzag64 => 64,
            .Zigzag256 => 256,
        };
    }

    const morton_index_map_1 = [2][2]u2{
        .{ 0x0, 0x1 },
        .{ 0x2, 0x3 },
    };

    const morton_index_map_2 = [4][4]u4{
        .{ 0x0, 0x1, 0x4, 0x5 },
        .{ 0x2, 0x3, 0x6, 0x7 },
        .{ 0x8, 0x9, 0xC, 0xD },
        .{ 0xA, 0xB, 0xE, 0xF },
    };

    const morton_index_map_3 = [8][8]u6{
        .{ 0x00, 0x01, 0x04, 0x05, 0x10, 0x11, 0x14, 0x15 },
        .{ 0x02, 0x03, 0x06, 0x07, 0x12, 0x13, 0x16, 0x17 },
        .{ 0x08, 0x09, 0x0C, 0x0D, 0x18, 0x19, 0x1C, 0x1D },
        .{ 0x0A, 0x0B, 0x0E, 0x0F, 0x1A, 0x1B, 0x1E, 0x1F },
        .{ 0x20, 0x21, 0x24, 0x25, 0x30, 0x31, 0x34, 0x35 },
        .{ 0x22, 0x23, 0x26, 0x27, 0x32, 0x33, 0x36, 0x37 },
        .{ 0x28, 0x29, 0x2C, 0x2D, 0x38, 0x39, 0x3C, 0x3D },
        .{ 0x2A, 0x2B, 0x2E, 0x2F, 0x3A, 0x3B, 0x3E, 0x3F },
    };

    const spring_index_map_1 = [4][4]u4{
        .{ 0x0, 0x1, 0x2, 0x3 },
        .{ 0x4, 0x5, 0x6, 0x7 },
        .{ 0x8, 0x9, 0xA, 0xB },
        .{ 0xC, 0xD, 0xE, 0xF },
    };

    const zigzag_index_map_1 = [4][4]u4{
        .{ 0x0, 0x1, 0x5, 0x6 },
        .{ 0x2, 0x4, 0x7, 0xC },
        .{ 0x3, 0x8, 0xB, 0xD },
        .{ 0x9, 0xA, 0xE, 0xF },
    };
};

const morton2 = getTiledLookup(.Morton2, u2, 2);
const morton2_fwd = morton2.forward;
const morton2_inv = morton2.reverse;
const morton4 = getTiledLookup(.Morton4, u4, 4);
const morton4_fwd = morton4.forward;
const morton4_inv = morton4.reverse;
const morton8 = getTiledLookup(.Morton8, u6, 8);
const morton8_fwd = morton8.forward;
const morton8_inv = morton8.reverse;
const morton16 = getTiledLookup(.Morton16, u8, 16);
const morton16_fwd = morton16.forward;
const morton16_inv = morton16.reverse;
const morton32 = getTiledLookup(.Morton32, u10, 32);
const morton32_fwd = morton32.forward;
const morton32_inv = morton32.reverse;
const morton64 = getTiledLookup(.Morton64, u12, 64);
const morton64_fwd = morton64.forward;
const morton64_inv = morton64.reverse;

const spring4 = getTiledLookup(.Spring4, u4, 4);
const spring4_fwd = spring4.forward;
const spring4_inv = spring4.reverse;
const spring16 = getTiledLookup(.Spring16, u8, 16);
const spring16_fwd = spring16.forward;
const spring16_inv = spring16.reverse;
const spring64 = getTiledLookup(.Spring64, u12, 64);
const spring64_fwd = spring64.forward;
const spring64_inv = spring64.reverse;

const zigzag4 = getTiledLookup(.Zigzag4, u4, 4);
const zigzag4_fwd = zigzag4.forward;
const zigzag4_inv = zigzag4.reverse;
const zigzag16 = getTiledLookup(.Zigzag16, u8, 16);
const zigzag16_fwd = zigzag16.forward;
const zigzag16_inv = zigzag16.reverse;
const zigzag64 = getTiledLookup(.Zigzag64, u12, 64);
const zigzag64_fwd = zigzag64.forward;
const zigzag64_inv = zigzag64.reverse;

/// Gets the curve index for the given coords; inverse of `getCoords` .
pub fn getIndex(comptime curve: Curve, row: GridIndex, col: GridIndex) Curve.index(curve) {
    return switch (curve) {
        .Morton2 => morton2_fwd[row][col],
        .Morton4 => morton4_fwd[row][col],
        .Morton8 => morton8_fwd[row][col],
        .Morton16 => morton16_fwd[row][col],
        .Morton32 => morton32_fwd[row][col],
        .Morton64 => morton64_fwd[row][col],
        .Spring4 => spring4_fwd[row][col],
        .Spring16 => spring16_fwd[row][col],
        .Spring64 => spring64_fwd[row][col],
        .Zigzag4 => zigzag4_fwd[row][col],
        .Zigzag16 => zigzag16_fwd[row][col],
        .Zigzag64 => zigzag64_fwd[row][col],
        else => getIndex2Stage(curve, row, col),
    };
}

/// Gets the grid coords for the given curve index; inverse of `getIndex` .
pub fn getCoords(comptime curve: Curve, index: Curve.index(curve)) [2]GridIndex {
    return switch (curve) {
        .Morton2 => morton2_inv[index],
        .Morton4 => morton4_inv[index],
        .Morton8 => morton8_inv[index],
        .Morton16 => morton16_inv[index],
        .Morton32 => morton32_inv[index],
        .Morton64 => morton64_inv[index],
        .Spring4 => spring4_inv[index],
        .Spring16 => spring16_inv[index],
        .Spring64 => spring64_inv[index],
        .Zigzag4 => zigzag4_inv[index],
        .Zigzag16 => zigzag16_inv[index],
        .Zigzag64 => zigzag64_inv[index],
        else => getCoords2Stage(curve, index),
    };
}

fn getIndex2Stage(comptime curve: Curve, row: GridIndex, col: GridIndex) Curve.index(curve) {
    const table = switch (curve) {
        .Morton32, .Morton64, .Morton128, .Morton256 => morton16_fwd,
        .Spring64, .Spring256 => spring16_fwd,
        .Zigzag64, .Zigzag256 => zigzag16_fwd,
        else => @compileError("2-stage lookup not supported for curve " ++ @tagName(curve)),
    };
    const row_hi = (row >> 4) & 0xF;
    const col_hi = (col >> 4) & 0xF;
    const curve_hi: Curve.index(curve) = table[row_hi][col_hi];
    const row_lo = row & 0xF;
    const col_lo = col & 0xF;
    const curve_lo: Curve.index(curve) = table[row_lo][col_lo];
    return (curve_hi << 8) | curve_lo;
}

fn getCoords2Stage(comptime curve: Curve, index: Curve.index(curve)) [2]GridIndex {
    const table = switch (curve) {
        .Morton64, .Morton128, .Morton256 => morton16_inv,
        .Spring64, .Spring256 => spring16_inv,
        .Zigzag64, .Zigzag256 => zigzag16_inv,
        else => @compileError("2-stage lookup not supported for curve " ++ @tagName(curve)),
    };
    const index_lo: u8 = @truncate(index);
    const index_hi: u8 = @truncate(index >> 8);
    const coords_lo = table[index_lo];
    const coords_hi = table[index_hi];
    const row = @as(GridIndex, coords_lo[0]) | (@as(GridIndex, coords_hi[0]) << 4);
    const col = @as(GridIndex, coords_lo[1]) | (@as(GridIndex, coords_hi[1]) << 4);
    return .{ row, col };
}

// NOTE: the below constructs a huge table if used for deep trees.
// This results in slow compile times (and may add to cache pressure at runtime).
// Recommend it's only used for computing tables upto 64x64 dimensions.
// For deeper trees use the 2-stage lookup functions.
fn getTiledLookup(
    comptime curve: Curve,
    comptime Index: type,
    comptime n: usize,
) struct { forward: [n][n]Index, reverse: [n * n][2]GridIndex } {
    @setEvalBranchQuota(200_000);
    const base = comptime Curve.base(curve);
    const levels = comptime (math.log2_int(GridIndex, n) / math.log2_int(GridIndex, base));
    const axis_bitshift = math.log2_int(u8, base);
    const mask = @as(usize, base) - 1;
    var forward: [n][n]Index = undefined;
    var reverse: [n * n][2]GridIndex = undefined;
    for (0..n) |row| {
        for (0..n) |col| {
            var index: Index = 0;
            inline for (0..levels) |lvl| {
                const remaining = @as(usize, levels) - lvl - 1;
                const axis_shift = remaining * axis_bitshift;
                const lvl_row = (row >> @truncate(axis_shift)) & mask;
                const lvl_col = (col >> @truncate(axis_shift)) & mask;
                const lvl_index = getPartialIndex(curve, 1, Index, lvl_row, lvl_col);
                const index_shift = 2 * remaining * axis_bitshift;
                index |= lvl_index << @intCast(index_shift);
            }
            forward[row][col] = index;
            reverse[index] = .{ @truncate(row), @truncate(col) };
        }
    }
    return .{ .forward = forward, .reverse = reverse };
}

// Gets a partial index by applying a lookup table up to 8 bits long.
fn getPartialIndex(
    comptime curve: Curve,
    comptime levels: u4,
    comptime Index: type,
    row: usize,
    col: usize,
) Index {
    return switch (curve) {
        .Morton2, .Morton4, .Morton8, .Morton16, .Morton32, .Morton64, .Morton128, .Morton256 => switch (levels) {
            1 => @intCast(Curve.morton_index_map_1[row][col]),
            2 => @intCast(Curve.morton_index_map_2[row][col]),
            3 => @intCast(Curve.morton_index_map_3[row][col]),
            4 => @intCast(Curve.morton_index_map_4[row][col]),
            else => @compileError("Morton lookup only supports 1-4 levels"),
        },

        .Spring4, .Spring16, .Spring64, .Spring256 => switch (levels) {
            1 => @intCast(Curve.spring_index_map_1[row][col]),
            2 => @intCast(Curve.spring_index_map_2[row][col]),
            else => @compileError("Spring lookup only supports 1-2 levels"),
        },

        .Zigzag4, .Zigzag16, .Zigzag64, .Zigzag256 => switch (levels) {
            1 => @intCast(Curve.zigzag_index_map_1[row][col]),
            2 => @intCast(Curve.zigzag_index_map_2[row][col]),
            else => @compileError("Zigzag lookup only supports 1-2 levels"),
        },
    };
}

const testing = std.testing;

test "index and coords round trip" {
    const curves = [_]Curve{
        .Morton2,
        .Morton4,
        .Morton8,
        .Morton16,
        .Morton32,
        .Morton64,
        .Morton128,
        .Morton256,
        .Spring4,
        .Spring16,
        .Spring64,
        .Spring256,
        .Zigzag4,
        .Zigzag16,
        .Zigzag64,
        .Zigzag256,
    };
    inline for (curves) |curve| {
        const n = Curve.size(curve);
        for (0..n) |row| {
            for (0..n) |col| {
                const index = getIndex(curve, @intCast(row), @intCast(col));
                const coords = getCoords(curve, index);
                try testing.expectEqual(row, coords[0]);
                try testing.expectEqual(col, coords[1]);
            }
        }
    }
}
