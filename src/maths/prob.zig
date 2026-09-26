const std = @import("std");
const Vec2f = @Vector(2, f32);

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
