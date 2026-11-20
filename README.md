# raytracing-one-weekend-go

My Go port of ray tracing in one weekend. This is my first ever Go project, I tried to get fluent with basics and tried to follow standard practices and naming conventions as I understood them. 

[_Ray Tracing in One Weekend_](https://raytracing.github.io/books/RayTracingInOneWeekend.html)

## BVH acceleration

Scenes with thousands of spheres are slow when every ray linearly tests every
object. The scene now also supports a bounding volume hierarchy (BVH); the
nearest-hit result is identical to the linear traversal, and occlusion and
material scattering are unchanged.

### Traversal modes

```sh
# BVH traversal (default)
go run . -accel=bvh  > image.ppm
# Original linear traversal, kept selectable for comparison
go run . -accel=list > image.ppm
```

Other flags: `-seed` (fixed RNG seed, default 20240101), `-render=sync|goroutines`,
`-grid` (denser scenes via larger values, 11 reproduces the original scene),
`-width`, `-samples`.

### BVH split rule (deterministic)

1. Compute the bounding box of every object's centroid and choose its longest
   axis (ties resolved X, then Y, then Z).
2. Stably sort objects by centroid coordinate on that axis (stable sort keeps
   insertion order for coincident objects, so equal-distance tie-breaking and
   the whole tree are reproducible).
3. Split the sorted slice in half; two-object nodes become leaves, single
   objects become leaf nodes.

### Edge-case handling

- **Empty scene**: `BuildBVH` returns `ErrEmptyScene`; the empty list remains a
  valid scene whose `Hit` always misses.
- **Coincident spheres**: stable sort preserves insertion order; equal-`t` hits
  keep the first inserted object, in both traversal modes.
- **Tiny / zero / huge radii**: point-like spheres get padded bounding boxes;
  zero-radius hits use a facing normal instead of dividing by zero; tiny radii
  normalize their normal explicitly to avoid floating-point drift; negative
  radii clamp to zero.
- **Near-parallel rays**: the AABB slab test uses inverse directions and
  handles zero direction components and NaN explicitly.
- **Unbounded objects**: primitives without a finite box (e.g. infinite-radius
  spheres) report `ok == false` from `Bound` and `BuildBVH` returns
  `ErrUnboundedObject`; they can still be rendered through the plain list.

### Reproducing the results

```sh
go test ./...                                         # all correctness tests
go test ./hittable/ -bench BenchmarkTraversal         # sparse vs dense traversal
go test ./camera/ -bench BenchmarkSceneRenderListVsBVH  # end-to-end render
```

On the reference machine the dense (~1900 object) fixed-seed scene performs
~440x fewer primitive intersection tests per ray, and an end-to-end single-
thread render is about 10x faster than the linear list, with bit-identical
pixels (`-render=sync`, same seed).
