//! Helper module for producing visuals for testing / debugging.
const std = @import("std");
const mem = std.mem;
const Vec2f = @Vector(2, f32);

/// Used to compose images from basic shapes; useful for eyeballing test data.
pub const Canvas = struct {
    min: Vec2f,
    max: Vec2f,
    str: std.ArrayList(u8),
    styles: std.StringHashMap(u16),
    style_str: std.ArrayList(u8),
    const Self = @This();

    pub fn init(
        allocator: mem.Allocator,
        min: Vec2f,
        max: Vec2f,
        background_style: Style,
    ) !Self {
        var str = try std.ArrayList(u8).initCapacity(allocator, 2000);
        errdefer str.deinit(allocator);
        var style_str = try std.ArrayList(u8).initCapacity(allocator, 512);
        errdefer style_str.deinit(allocator);
        var canvas: Self = .{
            .min = min,
            .max = max,
            .str = str,
            .styles = std.StringHashMap(u16).init(allocator),
            .style_str = style_str,
        };
        errdefer {
            var keys = canvas.styles.keyIterator();
            while (keys.next()) |key| allocator.free(key.*);
            canvas.styles.deinit();
        }
        try canvas.addRectangle(allocator, min, max, background_style);
        return canvas;
    }

    pub fn deinit(self: *Self, allocator: mem.Allocator) void {
        var keys = self.styles.keyIterator();
        while (keys.next()) |key| allocator.free(key.*);
        self.styles.deinit();
        self.style_str.deinit(allocator);
        self.str.deinit(allocator);
    }

    fn getClass(self: *Self, allocator: mem.Allocator, elt_style: []const u8) !u16 {
        if (self.styles.get(elt_style)) |class| return class;
        const class: u16 = @intCast(self.styles.count());
        const key = try allocator.dupe(u8, elt_style);
        errdefer allocator.free(key);
        var buf: [32]u8 = undefined;
        const class_start = try std.fmt.bufPrint(&buf, "\n.s{d}{{", .{class});
        try self.style_str.appendSlice(allocator, class_start);
        var tokens = mem.tokenizeScalar(u8, elt_style, ' ');
        while (tokens.next()) |token| {
            const eq = mem.indexOfScalar(u8, token, '=') orelse continue;
            try self.style_str.appendSlice(allocator, token[0..eq]);
            try self.style_str.append(allocator, ':');
            try self.style_str.appendSlice(allocator, token[eq + 2 .. token.len - 1]);
            try self.style_str.append(allocator, ';');
        }
        try self.style_str.append(allocator, '}');
        try self.styles.put(key, class);
        return class;
    }

    pub fn addLine(
        self: *Self,
        allocator: mem.Allocator,
        start: Vec2f,
        end: Vec2f,
        style: Style,
    ) !void {
        var buf: [256]u8 = undefined;
        const style_str = try style.getElementString(&buf);
        const class = try self.getClass(allocator, style_str);
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<line x1=\"{d:.3}\" y1=\"{d:.3}\" x2=\"{d:.3}\" y2=\"{d:.3}\" class=\"s{d}\"/>",
            .{ start[0], start[1], end[0], end[1], class },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addPolyline(
        self: *Self,
        allocator: mem.Allocator,
        points: []Vec2f,
        style: Style,
    ) !void {
        var sbuf: [128]u8 = undefined;
        const style_str = try style.getElementString(&sbuf);
        const class = try self.getClass(allocator, style_str);
        var pts_char_list = try std.ArrayList(u8).initCapacity(allocator, points.len * 12);
        defer pts_char_list.deinit(allocator);
        for (points) |p| {
            var buf: [32]u8 = undefined;
            const slice = buf[0..];
            const str = try std.fmt.bufPrint(slice, "{d:.3},{d:.3} ", .{ p[0], p[1] });
            try pts_char_list.appendSlice(allocator, str);
        }
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<polyline points=\"{s}\" class=\"s{d}\"/>",
            .{ pts_char_list.items[0..], class },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addRectangle(
        self: *Self,
        allocator: mem.Allocator,
        start: Vec2f,
        end: Vec2f,
        style: Style,
    ) !void {
        var buf: [128]u8 = undefined;
        const style_str = try style.getElementString(&buf);
        const class = try self.getClass(allocator, style_str);
        const d = end - start;
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<rect x=\"{d:.3}\" y=\"{d:.3}\" width=\"{d:.3}\" height=\"{d:.3}\" class=\"s{d}\"/>",
            .{ start[0], start[1], d[0], d[1], class },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addCircle(
        self: *Self,
        allocator: mem.Allocator,
        centre: Vec2f,
        radius: f32,
        style: Style,
    ) !void {
        var buf: [128]u8 = undefined;
        const style_str = try style.getElementString(&buf);
        const class = try self.getClass(allocator, style_str);
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<circle cx=\"{d:.3}\" cy=\"{d:.3}\" r=\"{d:.3}\" class=\"s{d}\"/>",
            .{ centre[0], centre[1], radius, class },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addPolygon(
        self: *Self,
        allocator: mem.Allocator,
        points: []Vec2f,
        style: Style,
    ) !void {
        var sbuf: [128]u8 = undefined;
        const style_str = try style.getElementString(&sbuf);
        const class = try self.getClass(allocator, style_str);
        var pts_char_list = try std.ArrayList(u8).initCapacity(allocator, points.len * 12);
        defer pts_char_list.deinit(allocator);
        for (points) |p| {
            var buf: [32]u8 = undefined;
            const slice = buf[0..];
            const str = try std.fmt.bufPrint(slice, "{d:.3},{d:.3} ", .{ p[0], p[1] });
            try pts_char_list.appendSlice(allocator, str);
        }
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<polygon points=\"{s}\" class=\"s{d}\"/>",
            .{ pts_char_list.items[0..], class },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    pub fn addText(
        self: *Self,
        allocator: mem.Allocator,
        centre: Vec2f,
        text: []const u8,
        font_size: f32,
        fill_hsl: [3]u16,
    ) !void {
        var buf: [256]u8 = undefined;
        const style_str = try std.fmt.bufPrint(
            &buf,
            "fill=\"hsl({d:.0},{d:.0}%,{d:.0}%)\" text-anchor=\"middle\"",
            .{ fill_hsl[0], fill_hsl[1], fill_hsl[2] },
        );
        const class = try self.getClass(allocator, style_str);
        const element_str = try std.fmt.allocPrint(
            allocator,
            "\n<text x=\"{d:.3}\" y=\"{d:.3}\" font-size=\"{d:.4}\" class=\"s{d}\">{s}</text>",
            .{ centre[0], centre[1], font_size, class, text },
        );
        defer allocator.free(element_str);
        try self.str.appendSlice(allocator, element_str);
    }

    // caller owns the returned memory
    pub fn getSvg(self: *const Self, allocator: mem.Allocator) ![]u8 {
        const extent = self.max - self.min;
        const rendered_max_dim: f32 = 800.0;
        const safe_w = if (extent[0] != 0) extent[0] else 1.0;
        const safe_h = if (extent[1] != 0) extent[1] else 1.0;
        const aspect = safe_w / safe_h;
        const rendered_w = if (aspect >= 1.0) rendered_max_dim else rendered_max_dim * aspect;
        const rendered_h = if (aspect >= 1.0) rendered_max_dim / aspect else rendered_max_dim;
        const xmlns_elt = "xmlns=\"http://www.w3.org/2000/svg\"";
        const svg_start = try std.fmt.allocPrint(
            allocator,
            "<svg viewBox=\"{d:.3} {d:.3} {d:.3} {d:.3}\" width=\"{d:.3}\" height=\"{d:.3}\" {s}>",
            .{ self.min[0], self.min[1], extent[0], extent[1], rendered_w, rendered_h, xmlns_elt },
        );
        defer allocator.free(svg_start);

        const sum_lengths = svg_start.len + self.style_str.items.len + self.str.items.len;
        var text = try std.ArrayList(u8).initCapacity(allocator, sum_lengths);
        try text.appendSlice(allocator, svg_start);
        try text.appendSlice(allocator, "\n<style>");
        try text.appendSlice(allocator, self.style_str.items);
        try text.appendSlice(allocator, "\n</style>");
        try text.appendSlice(allocator, self.str.items);
        try text.appendSlice(allocator, "\n</svg>");

        return text.toOwnedSlice(allocator);
    }

    pub fn writeToFile(
        self: *Self,
        io: std.Io,
        allocator: mem.Allocator,
        filename: []const u8,
        wrap_html: bool,
    ) !void {
        const wrap_start = if (wrap_html) "<!DOCTYPE html>\n<html><body>" else "";
        const wrap_end = if (wrap_html) "</body></html>" else "";
        const svg_body = try self.getSvg(allocator);
        defer allocator.free(svg_body);

        if (std.fs.path.dirname(filename)) |dir| {
            try std.Io.Dir.cwd().createDirPath(io, dir);
        }
        var file = try std.Io.Dir.cwd().createFile(io, filename, .{});
        defer file.close(io);
        var buf: [4096]u8 = undefined;
        var buf_writer = file.writer(io, &buf);
        var writer = &buf_writer.interface;
        try writer.print("{s}\n{s}\n{s}", .{ wrap_start, svg_body, wrap_end });
        try buf_writer.flush();
    }
};

pub const Style = struct {
    fill_active: bool = false,
    fill_hsl: [3]u16 = .{ 0, 0, 0 },
    fill_opacity: f32 = 1.0,
    stroke_active: bool = true,
    stroke_hsl: [3]u16 = .{ 0, 0, 0 },
    stroke_width: f32 = 1,
    stroke_opacity: f32 = 1.0,
    stroke_dashed: bool = false,

    pub fn getElementString(style: *const Style, buf_ptr: []u8) ![]u8 {
        var len: usize = 0;
        if (style.fill_active) {
            len = (try std.fmt.bufPrint(
                buf_ptr,
                "fill=\"hsl({d:.0},{d:.0}%,{d:.0}%)\" ",
                .{ style.fill_hsl[0], style.fill_hsl[1], style.fill_hsl[2] },
            )).len;
        } else {
            len = (try std.fmt.bufPrint(
                buf_ptr,
                "fill=\"none\" ",
                .{},
            )).len;
        }
        if (style.stroke_active) {
            len += (try std.fmt.bufPrint(
                buf_ptr[len..],
                "stroke=\"hsl({d:.0},{d:.0}%,{d:.0}%)\" stroke-width=\"{d:.4}\" ",
                .{ style.stroke_hsl[0], style.stroke_hsl[1], style.stroke_hsl[2], style.stroke_width },
            )).len;

            if (style.stroke_opacity < 1.0) {
                len += (try std.fmt.bufPrint(
                    buf_ptr[len..],
                    "stroke-opacity=\"{d:.4}\" ",
                    .{style.stroke_opacity},
                )).len;
            }
            if (style.stroke_dashed) {
                len += (try std.fmt.bufPrint(
                    buf_ptr[len..],
                    "stroke-dasharray=\"{},{}\" ",
                    .{ 2 * style.stroke_width, 2 * style.stroke_width },
                )).len;
            }
        }
        return buf_ptr[0..len];
    }
};
