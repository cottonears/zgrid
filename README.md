# Zgrid

Zgrid is a library for 2D spatial queries that strives to be simple, lightweight, and efficient.
It supports the following primitives: axis-aligned bounding-boxes (AABBs), oriented bounding-boxes (OBBs), balls, and lines.

Currently zgrid only offers one data structure, `SquareTree`, for queries in 2D scenes.
A square tree is ideal for for realtime applications where objects are densely packed (e.g., life / particle simulators, RPG-/RTS-style games).
It should scale well for scenes with 10,000+ objects (more with multi-threading).
A square tree won't be suitable for every application, it is likely to be slower than alternatives in sparse scenes.
Other types of trees may be implemented in future, see the [Roadmap](#roadmap).

## Prerequisites
Zig 0.16.

## Installation

Use `zig fetch` to import the zgrid package into your project:
``` sh
zig fetch --save git+https://github.com/cottonears/zgrid
```
Then add it as a dependency step in your `build.zig` file:
``` zig
const zgrid_dep = b.dependency("zgrid", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("zgrid", zgrid_dep.module("zgrid"));
```

## Basic working example

The code below shows how you can create a square tree, populate it, and perform spatial queries:

``` zig
const std = @import("std");
const zgrid = @import("zgrid");
const Ball2f = zgrid.Ball2f;
const Box2f = zgrid.Box2f;
const Line2f = zgrid.Line2f;
const Vec2f = zgrid.Vec2f; // i.e., @Vector(2, f32)
const SquareTree = zgrid.SquareTree(
    zgrid.Indexer2f(.Zigzag16, 1), // indexes 16 x 16 cells (quite coarse)
    Box2f, // primitive type to store in the tree
    u16, // used to identify your data
);
const Neighbour = SquareTree.Neighbour; // id type (u16) depends on above

pub fn main(init: std.process.Init) !void {
    const arena: std.mem.Allocator = init.arena.allocator();

    const min = Vec2f{ 0, 0 };
    const max = Vec2f{ 16, 12 };
    var tree = try SquareTree.init(arena, min, max, 64_000, 0);
    defer tree.deinit(arena);

    var pairs_buf: [1024][2]u16 = undefined; // used to record id-pairs in overlap queries
    var near_buf: [256]Neighbour = undefined; // used record neighbour info
    var entity_ids: [4]u16 = .{ 7, 25, 42, 1337 };
    var entity_aabbs: [4]Box2f = .{
        .{ .min = .{ 1.0, 1.0 }, .max = .{ 3.0, 5.0 } },
        .{ .min = .{ 8.5, 5.0 }, .max = .{ 9.5, 6.0 } },
        .{ .min = .{ 0.9, 4.8 }, .max = .{ 8.3, 5.9 } },
        .{ .min = .{ 5.3, 9.0 }, .max = .{ 17.0, 11.2 } },
    };
    var found: usize = 0;

    // square trees can be rebuilt cheaply: clear and update every frame
    tree.clear();
    try tree.addVolumes(entity_aabbs[0..], entity_ids[0..]);
    try tree.build();

    // check for overlaps betweeen stored objects with findSelfOverlaps
    const entity_pairs = try tree.findSelfOverlaps(pairs_buf[found..]);
    for (entity_pairs) |p| {
        std.debug.print("Entity overlap between {d} and {d}.\n", .{ p[0], p[1] });
    }
    found += entity_pairs.len;

    // check for overlaps with several external volumes with findExtOverlaps
    const line_ids: [2]u16 = .{ 100, 200 };
    const lines = [2]Line2f{
        .{ .start = .{ 0, 0 }, .end = .{ 8, 5 } },
        .{ .start = .{ 8, 5 }, .end = .{ 16, 0 } },
    };
    const line_pairs = try tree.findExtOverlaps(pairs_buf[found..], &line_ids, &lines);
    for (line_pairs) |p| {
        std.debug.print("Line {d} overlaps with {d}.\n", .{ p[0], p[1] });
    }
    found += line_pairs.len;

    // another external volume query: single volume method
    const query_ball = Ball2f{ .centre = .{ 4, 4 }, .radius = 3 };
    const ball_pairs = try tree.findExtOverlapsSingle(pairs_buf[found..], 0, query_ball);
    for (ball_pairs) |p| {
        std.debug.print("Query ball overlaps: {any}.\n", .{p});
    }
    found += line_pairs.len;

    // find the closest 2 volues to a single test point
    const test_pt = Vec2f{ 8, 6 };
    const nearby = try tree.findNeighboursSingle(&near_buf, test_pt, null, 3, 9.0);
    for (nearby) |n| {
        std.debug.print("Neighbour found: id = {d}, dist = {d:.3}.\n", .{ n.id, n.dist });
    }
}
```

There is a companion project [`zgrid-demo`](https://github.com/cottonears/zgrid-demo) that uses zgrid + SDL3 in a simple particle simulation.


## Volumes
Several types of volumes supported, describe them and contrast storable vs non-storable.
(add images!)


## SquareTree

![SquareTree-Ball](docs/img/square_tree_ball.svg)

A square tree is a uniform grid where each top-level (level 0) cell has a a bounding volume hierachy (BVH) tree beneath it.
Adding the BVH allows for more flexible queries and better performance if some cells become densely packed.
Cells are subdivided into 2x2 or 4x4 children depending on the chosen indexer.
The number of levels in the tree is also determined by the choice of indexer; see [Indexing](#indexing) for more details).

When volumes have been added to a square tree and `build` is called, the volumes are 'binned' into a leaf cells based on their centre positions.
Volumes whose centre is outside the tree's bounds will be binned into a cell on the edge of the tree (using clamp()). 
Bounding boxes are fit around all binned volumes on the leaf level, then combined to create one bounding box for each leaf node.
Following this, another layer of bounding boxes is created for the level above this, ... and so on.
Bounding volumes for each node may extend well outside the central cell for that node, and overlap with other bounding volumes on the same level. 
No attempt is made to prevent this: simplicity and speed of indexing + rebuilding are prioritised.

The square tree data structure uses a single slice to store all volumes (from all leaf cells) in a one large block of memory.
This has the following benefits:
- The tree allocates heap memory exactly once (on init). Adding volumes to the tree after initialisation will never trigger a heap allocation, but it may result in a `TreeCapacityExceeded` error (if the initial capacity has been exhausted).
- Volumes within the same leaf are stored in contiguous memory; this is important for performance.

The proportion of the tree's backing slice allocated to each leaf cell is variable and will adapt as required at runtime.
This is achieved by using offsets calculated in a counting sort during the `build` step to compactly partition the slice.
This results in lower memory usage (+ safer runtime behaviour) than a naiive approach where each cell is backed by a separate slice.


(A paragraph about querying the tree and BFS/DTT here)

## Indexing
(Write about the recursive indexing techniques used)

![Spring16](docs/img/curve_spring_16.svg)
![Morton16](docs/img/curve_morton_16.svg)
![ZigZag16](docs/img/curve_zigzag_16.svg)


## Sizing your square tree
(Tips + directions to how to size trees and use the benchmark tool on data representative of use-case, or (even better) directly imported data from a real scene).

## 0.1 TODO
- [X] Improve benchmarking reports + tooling (better stats + warmup queries).
- [X] Finish `findNearestNeighbours` (expanding-ring search).
- [X] Add `getLeafOccupancyUnderNode` + an indexer helper (e.g. `getLeafSuccessorRange`) to help with workload partitioning.
- [X] Implement helper for determining suitable number workers + parts for parallel methods.
- [X] Add findExtOverlapsSingle.
- [X] Improve indexing performance.
- [X] Parallelise build (with a radix sort?).
- [ ] Move benchmark to a separate repo to reduce compile times.
- [ ] Implement `getExpandedVolume(V, vol, velocity, time_step)` (makes conservative BVs for moving bodies); required to prevent tunnelling.
- [ ] Revamp this readme.
- [ ] Set up CI (`zig build test` on push).

## Roadmap
In no particular order:
- Allow lines to be stored? Could be useful and should be easy.
- Add support for convex hulls (definitely useful, not as easy).
- Research BIGMIN/LITMAX as potential performance improvements for neighbours search.
- Add layered_tree that wraps several trees (e.g., static + dynamic, player1, player2) and allows for easy in-tree and cross-tree queries.
- Experiment with a dynamic-depth 2D linear BVH along the lines of: https://research.nvidia.com/sites/default/files/pubs/2012-06_Maximizing-Parallelism-in/karras2012hpg_paper.pdf
