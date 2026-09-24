const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const mod = b.addModule("zgrid", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    const mod_tests = b.addTest(.{ .root_module = mod });
    const run_mod_tests = b.addRunArtifact(mod_tests);
    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_mod_tests.step);

    // TODO: bring this back but make sure it's not part of the regular build step
    // const bench_exe = b.addExecutable(.{
    //     .name = "zgrid-bench",
    //     // ...
    // });
    // const run_bench = b.addRunArtifact(bench_exe);
    // const bench_step = b.step("bench", "Run benchmarks");
    // bench_step.dependOn(&run_bench.step);
}
