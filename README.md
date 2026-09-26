# Zgrid

Zgrid is a lightweight library for fast 2D spatial queries in dynamic scenes.
It provides queries for overlap detection and and nearest-neighbour searches (measured centre-centre).
Several primitives are supported: axis-aligned bounding-boxes (AABBs), oriented bounding-boxes (OBBs), balls, and lines.

Zgrid currently provides one data structure, `SquareTree`, for queries in 2D scenes.
A square tree combines a regular grid with a bounding volume hierarchy (BVH), making it suitable for dynamic scenes with large numbers of objects.
Indexers can be chosen to make the tree behave more like a fine uniform grid, or to add further layers of hierarchy to accelerate queries where volumes are unevenly distributed or vary significantly in size.
Efficient, simple-to-use parallel methods are also provided alongside serial implementations.

Other spatial data structures are planned once the public API of `SquareTree` has stabilised; see the [Roadmap](#roadmap).


## Prerequisites
Zig 0.16.


## Installation

Run `zig fetch` to add zgrid to your project's `build.zig.zon`:
``` sh
zig fetch --save git+https://github.com/cottonears/zgrid
```
Then edit your `build.zig` file to add it as a dependency:
``` zig
const zgrid_dep = b.dependency("zgrid", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("zgrid", zgrid_dep.module("zgrid"));
```


## Basic working example

The code below shows how you can create a square tree, populate it with AABBs, and perform some spatial queries:

```zig
const std = @import("std");
const zgrid = @import("zgrid");
const Ball2f = zgrid.Ball2f;
const Box2f = zgrid.Box2f;
const Line2f = zgrid.Line2f;
const Vec2f = zgrid.Vec2f;

const SquareTree = zgrid.SquareTree(
    zgrid.Indexer2f(.Morton64, 1), // indexes a 64 x 64 grid
    Box2f, // primitive type to store in the tree
    u16, // client ID type - whatever works best for you
);
const Neighbour = SquareTree.Neighbour;

pub fn main(init: std.process.Init) !void {
    const arena = init.arena.allocator();
    const min = Vec2f{ 0, 0 };
    const max = Vec2f{ 16, 12 };
    const tree_capacity = 64_000; // whole-tree capacity
    const max_workers = 0; // will default to number of CPUs - 1
    
    var tree = try SquareTree.init(arena, min, max, tree_capacity, max_workers);
    defer tree.deinit(arena);
    var pairs_buf: [1024][2]u16 = undefined;
    var near_buf: [256]Neighbour = undefined;
    
    const entity_ids = [_]u16{ 7, 25, 42, 1337 };
    const entity_aabbs = [_]Box2f{
        .{ .min = .{ 1.0, 1.0 }, .max = .{ 3.0, 5.0 } },
        .{ .min = .{ 8.5, 5.0 }, .max = .{ 9.5, 6.0 } },
        .{ .min = .{ 0.9, 4.8 }, .max = .{ 8.3, 5.9 } },
        .{ .min = .{ 5.3, 9.0 }, .max = .{ 17.0, 11.2 } },
    };

    try tree.add(&entity_aabbs, &entity_ids);
    try tree.build();
    var found: usize = 0;

    // Find overlaps between stored volumes.
    const entity_pairs = try tree.findSelfOverlaps(pairs_buf[found..]);
    for (entity_pairs) |p| {
        std.debug.print("Entity overlap between {d} and {d}.\n", .{ p[0], p[1] });
    }
    found += entity_pairs.len;

    // Find overlaps between stored volumes and several external lines.
    const line_ids = [_]u16{ 100, 200 };
    const lines = [_]Line2f{
        .{ .start = .{ 0, 0 }, .end = .{ 8, 5 } },
        .{ .start = .{ 8, 5 }, .end = .{ 16, 0 } },
    };
    const line_pairs = try tree.findExtOverlaps(pairs_buf[found..], &line_ids, &lines);
    for (line_pairs) |p| {
        std.debug.print("Line {d} overlaps with {d}.\n", .{ p[0], p[1] });
    }

    // Find the three nearest stored volumes to a point.
    const test_pt = Vec2f{ 8, 6 };
    const nearby = try tree.findNeighboursSingle(&near_buf, test_pt, null, 3, 9.0);
    for (nearby) |n| {
        std.debug.print("Neighbour found: id = {d}, dist = {d:.3}.\n", .{ n.id, n.dist });
    }

    tree.clear(); // empties the tree
}
```
A `SquareTree` is designed to be rebuilt frequently rather than maintained incrementally.
A typical update cycle is:
``` zig
tree.clear(); // removes previous contents without releasing backing memory
try tree.add(volumes, ids); // stages new volumes (+ their ids)
try tree.build(); // indexes the volumes and builds the BVH structure
```
After clearing the tree, you can also `relocate` it to a new position at very low cost.

There is a companion project that demonstrates how zgrid can be used for a simple particle simulation: see [`zgrid-demo`](https://github.com/cottonears/zgrid-demo).


## Volumes

Zgrid provides several 2D primitives for spatial queries.
Their fields are all typed as `f32` or `Vec2f` (an alias for `@Vector(2, f32)`). 

| Name            | Size  | Query speed  |               
| --------------- | ----- | ------------ |
| `Ball2f`        | 12 B  | Fast         |
| `Box2f`         | 16 B  | Fast         |
| `Line2f`        | 16 B  | Average      |
| `OrientedBox2f` | 24 B  | Average      |

A `SquareTree` stores a single volume type, chosen at compile time.
Lines cannot be stored at present, though support for this may be added in future.
All implemented volumes can be used for external tree queries: regardless of the stored type.
For example, a tree containing `Box2f` volumes can be queried using balls or lines.
This approach allows the tree to remain specialised for its stored data, while still supporting mixed-type spatial queries.


## SquareTree

A `SquareTree` combines a regular grid with a hierarchy of bounding volumes - whose exact structure is determined by the chosen indexer.
Below is an example of a square tree that uses the `Spring16` curve for indexing and stores ball volumes:

![SquareTree-Ball](docs/img/square_tree_ball.svg)

The pictured square tree has two levels:
- Level 0 has a 4 x 4 grid labelled 0 - F (purple)
- Level 1 has a 16 x 16 grid labelled 00 - FF (blue)

Note that (in hexadecimal) the first digit of each child cell's index identifies its parent.
This follows from the use of recursive curves for indexing; see [Indexing](#indexing) for more details.

Volumes are staged when `add` is called: their geometric data and client IDs are recorded with no further processing.
Adding volumes is very cheap, so there is no parallel variant of this method.

When `build` is called, each staged volume is assigned to a leaf cell based on its centre.
Volumes are then sorted into their final positions in storage, before the BVH is built.
For each leaf node, a bounding box is fitted around all volumes assigned to it.
Parent bounds are then constructed bottom-up by combining the bounds of their children.
Node bounds are not constrained to their nominal grid cells and may overlap other nodes at the same level.
This is intentional: `SquareTree` prioritises fast indexing and rebuilding over maintaining tightly partitioned spatial bounds.

The square tree data structure uses a single slice to store all built volumes in one large block of memory.
This has the following benefits:
- The tree allocates heap memory only within `init`. Adding volumes after initialisation will never trigger an allocation, but may result in a `TreeCapacityExceeded` error if the tree's capacity is exhausted.
- Volumes within the same leaf are stored in contiguous memory: improving cache locality during queries.

The proportion of the tree's backing slice used by each leaf cell is variable and will adapt as required at runtime.
This is achieved by using offsets calculated in a counting sort during the `build` step to compactly partition the slice.
This results in lower memory usage (+ safer runtime behaviour) than a naive approach where each cell is backed by a separate slice.

Overlap queries traverse the tree from coarse nodes at level 0 towards fine nodes at the leaf level.
Since each node covers its children, its entire subtree can be skipped when its BV doesn't intersect the query volume.
`SquareTree` provides both self-overlap queries and external-overlap queries:
- `findSelfOverlaps` returns intersecting pairs among volumes stored in the tree.
- `findExtOverlaps` returns pairs for the query volumes against the volumes stored in the tree. The query volumes can be any of those listed in the [Volumes table](#volumes)

Neighbour queries use a simple expanding-ring search over nearby leaf cells.
They return an ordered slice of ID + distance where the volume with the closest centre appears first.

Capacity limits:
- Tree capacity is limited to 16,777,215 volumes.
- Leaf nodes can hold at most 65,535 volumes each.


## Indexing

The choice of indexer is the most important parameter when using a `SquareTree`.
``` zig
const Indexer = zgrid.Indexer2f(.Morton64, 1);
```
The above defines an indexer that uses a Lebesgue / Morton curve to index a 64 x 64 grid.
This curve has a 2 x 2 pattern that is repeated recursively across the grid, so the resulting BVH is a quad tree.
The pattern must be applied recursively 6 times to tile the grid (since $2^6 = 64$), so the tree has an effective depth of 6.

The second parameter controls *compression*: how many levels of the hierarchy are collapsed into the top level.
Using a compression of 1 will result in an uncompressed tree, so in the above example level 0 is a 2 x 2 grid.
If the compression is set to a higher number, the top-level grid becomes finer and only the hierarchy in lower levels is retained.
If compression is set equal to the tree's effective depth (6 in the above example), then the entire hierarchy will be collapsed into level 0 - the tree becomes a uniform grid.

Aside from the grid-size and tree depth ramifications, the choice of curve makes little practical difference (at present).
All curves have possess same key recursive property, although there are marginal differences in how well they preserve locality.
It's conceivable that future algorithms may perform better with some curves than others (due to the locality differences), but this remains to be seen.
Another reason for keeping them around is that they are pretty to look at; see below.

### Morton (the standard option - a good default choice)
![Morton16](docs/img/curve_morton_16.svg)

### Spring (simple - and bouncy)
![Spring16](docs/img/curve_spring_16.svg)

### ZigZag (as popularised py jpeg)
![ZigZag16](docs/img/curve_zigzag_16.svg)


## Multi-threading

The core query methods are provided in `Parallel` variants alongside their serial counterparts.
These methods manage threading internally; they are lock-free and designed to balance their workloads evenly.

When using a `SquareTree`, its `max_workers` must be provided on init since it needs to allocate per-worker buffers for queries:
``` zig
const max_workers = 0; // 0 defaults to number CPUs - 1
var tree = try SquareTree.init(allocator, min, max, capacity, max_workers);
```

If your application has dedicated threads for concurrent processing (e.g., rendering, audio), you may want to choose a more conservative `max_workers` limit to avoid interfering with them.

Actually using the parallel methods is straightforward.
Aside from an extra `std.Io` parameter, everything remains the same:
``` zig
try tree.build();
const pairs = try tree.findExtOverlaps(pairs_buf[found..], &query_ids, &query_vols);
``` 
The parallel version of the above is:
``` zig
try tree.buildParallel(io);
const pairs = try tree.findExtOverlapsParallel(io, pairs_buf[found..], &query_ids, &query_vols);
``` 
Note the serial and parallel overlap queries will not return pairs in the same order.
If pair ordering is important in your application, you will need to sort the results after querying.

Concurrent calls on the same tree from different client threads is strongly discouraged.
Overlap queries use an internal scratch buffer and are not thread safe.
This includes the overlap queries, which modify internal scratch buffers while searching the tree.


## 0.1 TODO
- [X] Improve benchmarking reports + tooling (better stats + warmup queries).
- [X] Finish `findNearestNeighbours` (expanding-ring search).
- [X] Add `getLeafOccupancyUnderNode` + an indexer helper (e.g. `getLeafSuccessorRange`) to help with workload partitioning.
- [X] Implement helper for determining suitable number of workers + parts for parallel methods.
- [X] Add findExtOverlapsSingle.
- [X] Improve indexing performance.
- [X] Parallelise build (with a radix sort?).
- [X] Move benchmark to a separate repo to reduce compile times.
- [X] Bring the benchmark back in a way that won't affect importers' compile times.
- [X] Look into what is going on with the volume alignment + sizes, may need to go to scalar floats or simple arrays.
- [ ] Implement `vol.getExpanded(translation)` (makes conservative BVs for moving bodies); helper to avoid tunnelling.
- [x] Revamp this readme.
- [ ] Set up CI (`zig build test` on push).

## Roadmap
In no particular order:
- Allow lines to be stored? Could be useful and should be easy.
- Add support for convex hulls (definitely useful, not as easy).
- Try out dual-tree traversal (again) for self-overlap queries.
- Research BIGMIN/LITMAX as potential performance improvements for neighbours search.
- Implement optimised Morton indexing with PDEP + PEXT (with fallback to current LUT if not available).
- Add layered_tree that wraps several trees (e.g., static + dynamic) and allows for easy in-tree and cross-tree queries.
- Experiment with a dynamic-depth 2D linear BVH along the lines of: https://research.nvidia.com/sites/default/files/pubs/2012-06_Maximizing-Parallelism-in/karras2012hpg_paper.pdf
