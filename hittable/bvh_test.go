package hittable

import (
	"errors"
	"math"
	"math/rand"
	"testing"

	"github.com/davidelettieri/raytracing-one-weekend-go/ray"
	"github.com/davidelettieri/raytracing-one-weekend-go/utils"
	"github.com/davidelettieri/raytracing-one-weekend-go/vec"
)

// countingHittable wraps a primitive and counts how many times its Hit method
// is invoked, i.e. how many primitive intersection tests a traversal performs.
type countingHittable struct {
	inner Hittable
	calls int
}

func (c *countingHittable) Hit(r ray.Ray, interval utils.Interval) (HitRecord, bool) {
	c.calls++
	return c.inner.Hit(r, interval)
}

func (c *countingHittable) Bound() (AABB, bool) {
	return c.inner.Bound()
}

// unboundedHittable simulates a primitive without a finite bounding box, such
// as an infinite plane or an infinite-radius sphere.
type unboundedHittable struct{}

func (unboundedHittable) Hit(ray.Ray, utils.Interval) (HitRecord, bool) {
	return HitRecord{}, false
}

func (unboundedHittable) Bound() (AABB, bool) {
	return EmptyAABB(), false
}

// buildRandomScene builds a multi-sphere scene with a local, fixed-seed RNG so
// the test data is identical on every run and independent of global RNG use.
func buildRandomScene(t testing.TB, grid int) HittableList {
	t.Helper()
	rng := rand.New(rand.NewSource(424242))
	randomFloat := func() float64 { return rng.Float64() }

	world := NewHittableList()
	world.Add(NewSphere(vec.NewPoint3(0, -1000, 0), 1000, NewLambertian(vec.NewColor(0.5, 0.5, 0.5))))

	refPoint := vec.NewPoint3(4, 0.2, 0)
	for a := -float64(grid); a < float64(grid); a++ {
		for b := -float64(grid); b < float64(grid); b++ {
			chooseMat := randomFloat()
			center := vec.NewPoint3(a+0.9*randomFloat(), 0.2, b+0.9*randomFloat())
			if center.Subtract(refPoint).Length() <= 0.9 {
				continue
			}

			var material Material
			switch {
			case chooseMat < 0.8:
				material = NewLambertian(vec.NewColor(randomFloat(), randomFloat(), randomFloat()))
			case chooseMat < 0.95:
				material = NewMetal(vec.NewColor(0.6, 0.7, 0.8), randomFloat()*0.5)
			default:
				material = NewDielectric(1.5)
			}
			world.Add(NewSphere(center, 0.2, material))
		}
	}

	world.Add(NewSphere(vec.NewPoint3(0, 1, 0), 1.0, NewDielectric(1.5)))
	world.Add(NewSphere(vec.NewPoint3(-4, 1, 0), 1.0, NewLambertian(vec.NewColor(0.4, 0.2, 0.1))))
	world.Add(NewSphere(vec.NewPoint3(4, 1, 0), 1.0, NewMetal(vec.NewColor(0.7, 0.6, 0.5), 0.0)))
	return world
}

// fixedRays produces a deterministic ray set mixing camera-like view rays and
// edge-case rays (near-parallel to an axis, rays starting inside geometry).
func fixedRays(rng *rand.Rand, count int) []ray.Ray {
	rays := make([]ray.Ray, 0, count)
	origin := vec.NewPoint3(13, 2, 2)
	target := vec.NewPoint3(0, 0, 0)
	baseDirection := target.Subtract(origin).Unit()

	for i := range count {
		var direction vec.Vec3
		switch i % 5 {
		case 0:
			// Direction with a near-zero component: nearly parallel to a slab.
			direction = vec.NewVec3(1e-12, -0.25-rng.Float64()*0.1, -0.45-rng.Float64()*0.1)
		case 1:
			// Direction with an exactly zero component: parallel to a slab.
			direction = vec.NewVec3(0, -0.3, -0.5-rng.Float64()*0.2)
		case 2:
			// Ray starting inside the big ground sphere region.
			insideOrigin := vec.NewPoint3(rng.Float64()*2-1, 0.2, rng.Float64()*2-1)
			rays = append(rays, ray.NewRay(insideOrigin, vec.NewVec3(rng.Float64()-0.5, 1, rng.Float64()-0.5).Unit()))
			continue
		default:
			jitter := vec.NewVec3((rng.Float64()-0.5)*0.08, (rng.Float64()-0.5)*0.08, (rng.Float64()-0.5)*0.08)
			direction = baseDirection.Add(jitter).Unit()
		}
		rays = append(rays, ray.NewRay(origin, direction))
	}
	return rays
}

func sameHitRecord(a, b HitRecord) bool {
	return a.t == b.t &&
		a.frontFace == b.frontFace &&
		a.p.X() == b.p.X() && a.p.Y() == b.p.Y() && a.p.Z() == b.p.Z() &&
		a.normal.X() == b.normal.X() && a.normal.Y() == b.normal.Y() && a.normal.Z() == b.normal.Z() &&
		a.material == b.material
}

func TestBVHMatchesListNearestHit(t *testing.T) {
	for _, grid := range []int{2, 6, 11, 22} {
		world := buildRandomScene(t, grid)
		bvh, err := world.BuildBVH()
		if err != nil {
			t.Fatalf("grid=%d: unexpected build error: %v", grid, err)
		}

		rng := rand.New(rand.NewSource(99))
		rays := fixedRays(rng, 2000)
		hitInterval := utils.NewInterval(0.001, math.Inf(1))

		checked := 0
		for _, r := range rays {
			listRecord, listHit := world.Hit(r, hitInterval)
			bvhRecord, bvhHit := bvh.Hit(r, hitInterval)

			if listHit != bvhHit {
				t.Fatalf("grid=%d: hit mismatch (list=%v bvh=%v) for ray %v", grid, listHit, bvhHit, r)
			}
			if listHit && !sameHitRecord(listRecord, bvhRecord) {
				t.Fatalf("grid=%d: nearest-hit record mismatch\nt list=%+v\nt bvh =%+v", grid, listRecord, bvhRecord)
			}
			checked++
		}
		t.Logf("grid=%d: %d objects, %d rays verified identical", grid, world.Len(), checked)
	}
}

func TestBVHReducesIntersectionTests(t *testing.T) {
	world := buildRandomScene(t, 22)
	objects := world.Objects()

	counters := make([]*countingHittable, len(objects))
	listWrapped := NewHittableList()
	wrappedForBVH := make([]Hittable, len(objects))
	for i, obj := range objects {
		counter := &countingHittable{inner: obj}
		counters[i] = counter
		listWrapped.Add(counter)
		wrappedForBVH[i] = counter
	}

	bvh, err := NewBVH(wrappedForBVH)
	if err != nil {
		t.Fatalf("unexpected build error: %v", err)
	}

	rng := rand.New(rand.NewSource(7))
	rays := fixedRays(rng, 500)
	hitInterval := utils.NewInterval(0.001, math.Inf(1))

	resetCounters := func() {
		for _, c := range counters {
			c.calls = 0
		}
	}
	sumCounters := func() int {
		total := 0
		for _, c := range counters {
			total += c.calls
		}
		return total
	}

	listTotal, bvhTotal := 0, 0
	for _, r := range rays {
		resetCounters()
		listRecord, listHit := listWrapped.Hit(r, hitInterval)
		listTotal += sumCounters()

		resetCounters()
		bvhRecord, bvhHit := bvh.Hit(r, hitInterval)
		bvhTotal += sumCounters()

		if listHit != bvhHit || (listHit && !sameHitRecord(listRecord, bvhRecord)) {
			t.Fatalf("nearest-hit mismatch while counting")
		}
	}

	// The linear list always tests every object per ray.
	if listTotal != len(rays)*len(objects) {
		t.Fatalf("list test count = %d, want %d", listTotal, len(rays)*len(objects))
	}
	if bvhTotal >= listTotal {
		t.Fatalf("BVH performed %d tests, list %d: expected a reduction", bvhTotal, listTotal)
	}
	t.Logf("dense scene: %d objects, %d rays | list tests=%d bvh tests=%d (%.1fx fewer)",
		len(objects), len(rays), listTotal, bvhTotal, float64(listTotal)/float64(bvhTotal))
}

func TestBVHEmptyScene(t *testing.T) {
	empty := NewHittableList()
	bvh, err := empty.BuildBVH()
	if !errors.Is(err, ErrEmptyScene) {
		t.Fatalf("expected ErrEmptyScene, got bvh=%v err=%v", bvh, err)
	}

	// The empty list itself remains a valid, always-missing Hittable.
	r := ray.NewRay(vec.NewPoint3(0, 0, 0), vec.NewVec3(1, 0, 0))
	if _, hit := empty.Hit(r, utils.UniverseInterval()); hit {
		t.Fatal("empty scene must never report a hit")
	}
	if _, ok := empty.Bound(); ok {
		t.Fatal("empty scene must report no bounding box")
	}
}

func TestBVHUnboundedObject(t *testing.T) {
	world := NewHittableList()
	world.Add(NewSphere(vec.NewPoint3(0, 0, 0), 1, NewLambertian(vec.NewColor(1, 1, 1))))
	world.Add(unboundedHittable{})

	if _, err := world.BuildBVH(); !errors.Is(err, ErrUnboundedObject) {
		t.Fatalf("expected ErrUnboundedObject, got %v", err)
	}

	// An infinite-radius sphere is itself unbounded.
	infinite := NewHittableList()
	infinite.Add(NewSphere(vec.NewPoint3(0, 0, 0), math.Inf(1), NewLambertian(vec.NewColor(1, 1, 1))))
	if _, err := infinite.BuildBVH(); !errors.Is(err, ErrUnboundedObject) {
		t.Fatalf("expected ErrUnboundedObject for infinite sphere, got %v", err)
	}
}

func TestCoincidentSpheres(t *testing.T) {
	// Fully overlapping spheres, inserted in a known order.
	center := vec.NewPoint3(0, 0, -3)
	materialA := NewLambertian(vec.NewColor(0.1, 0.2, 0.3))
	materialB := NewMetal(vec.NewColor(0.4, 0.5, 0.6), 0.1)
	materialC := NewDielectric(1.5)

	list := NewHittableList()
	list.Add(NewSphere(center, 1, materialA))
	list.Add(NewSphere(center, 1, materialB))
	list.Add(NewSphere(center, 1, materialC))

	bvh, err := list.BuildBVH()
	if err != nil {
		t.Fatalf("unexpected build error: %v", err)
	}

	rng := rand.New(rand.NewSource(13))
	hitInterval := utils.NewInterval(0.001, math.Inf(1))
	for range 200 {
		origin := vec.NewPoint3(rng.Float64()*6-3, rng.Float64()*4-2, rng.Float64()*2)
		r := ray.NewRay(origin, center.Subtract(origin).Unit())

		listRecord, listHit := list.Hit(r, hitInterval)
		bvhRecord, bvhHit := bvh.Hit(r, hitInterval)
		if listHit != bvhHit || (listHit && !sameHitRecord(listRecord, bvhRecord)) {
			t.Fatalf("coincident spheres: tie-breaking diverged")
		}
		// Equal-t ties must keep the first object in insertion order.
		if listHit && listRecord.Material() != materialA {
			t.Fatalf("expected insertion-order tie-breaking to keep materialA, got %v", listRecord.Material())
		}
	}
}

func TestExtremeRadii(t *testing.T) {
	material := NewLambertian(vec.NewColor(0.8, 0.8, 0.8))
	list := NewHittableList()
	tiny := NewSphere(vec.NewPoint3(2, 0.2, 0), 1e-10, material)
	huge := NewSphere(vec.NewPoint3(-10, -1e6-0.5, -5), 1e6, material)
	point := NewSphere(vec.NewPoint3(-2, 0.2, 0), 0, material)
	list.Add(tiny)
	list.Add(huge)
	list.Add(point)

	// A negative radius is clamped to a point primitive rather than storing a
	// sign-inverted inverse radius.
	negative := NewSphere(vec.NewPoint3(3, 0.2, 0), -0.5, material)
	if negative.radius != 0 || negative.radiusSquared != 0 {
		t.Fatalf("negative radius must clamp to zero, got radius=%v sq=%v", negative.radius, negative.radiusSquared)
	}
	list.Add(negative)

	for _, s := range []Sphere{tiny, huge, point} {
		box, ok := s.Bound()
		if !ok || box.IsEmpty() {
			t.Fatalf("finite sphere must have a valid box: %+v ok=%v", s, ok)
		}
		if math.IsNaN(box.Min().X()) || math.IsInf(box.Max().X(), 0) {
			t.Fatalf("finite sphere box must stay finite: %+v", box)
		}
	}

	bvh, err := list.BuildBVH()
	if err != nil {
		t.Fatalf("unexpected build error: %v", err)
	}

	hitInterval := utils.NewInterval(0.001, math.Inf(1))

	// A ray aimed directly at the tiny sphere centre must hit it in both modes.
	rTiny := ray.NewRay(vec.NewPoint3(2, 0.2, 5), vec.NewVec3(0, 0, -1))
	listRecord, listHit := list.Hit(rTiny, hitInterval)
	bvhRecord, bvhHit := bvh.Hit(rTiny, hitInterval)
	if !listHit || !bvhHit || !sameHitRecord(listRecord, bvhRecord) {
		t.Fatalf("tiny sphere hit mismatch: list=%v bvh=%v", listHit, bvhHit)
	}

	// A ray tangent to the point sphere (zero radius) must not panic and must
	// agree across traversal modes.
	rPoint := ray.NewRay(vec.NewPoint3(-2, 0.2, 5), vec.NewVec3(0, 0, -1))
	listRecord, listHit = list.Hit(rPoint, hitInterval)
	bvhRecord, bvhHit = bvh.Hit(rPoint, hitInterval)
	if listHit != bvhHit || (listHit && !sameHitRecord(listRecord, bvhRecord)) {
		t.Fatalf("point sphere tangent hit mismatch: list=%v bvh=%v", listHit, bvhHit)
	}

	// A ray crossing the huge sphere from outside its radius.
	hugeCenter := vec.NewPoint3(-10, -1e6-0.5, -5)
	rHuge := ray.NewRay(hugeCenter.Add(vec.NewPoint3(0, 2e6, 0)), vec.NewVec3(0, -1, 0))
	listRecord, listHit = list.Hit(rHuge, hitInterval)
	bvhRecord, bvhHit = bvh.Hit(rHuge, hitInterval)
	if !listHit || !bvhHit || !sameHitRecord(listRecord, bvhRecord) {
		t.Fatalf("huge sphere hit mismatch: list=%v bvh=%v", listHit, bvhHit)
	}
}

func TestAABBSlabEdgeCases(t *testing.T) {
	box := NewAABB(vec.NewPoint3(-1, -1, -1), vec.NewPoint3(1, 1, 1))
	interval := utils.NewInterval(0, math.Inf(1))

	// Ray parallel to an axis, outside the slab: miss.
	parallelMiss := ray.NewRay(vec.NewPoint3(5, 0, 0), vec.NewVec3(0, 1, 0))
	if _, hit := box.Hit(parallelMiss, interval); hit {
		t.Fatal("parallel ray outside the slab must miss")
	}

	// Ray parallel to an axis, inside the slab: hit.
	parallelHit := ray.NewRay(vec.NewPoint3(0, 0, 0), vec.NewVec3(0, 1, 0))
	if _, hit := box.Hit(parallelHit, interval); !hit {
		t.Fatal("parallel ray inside the slab must hit")
	}

	// Ray starting inside the box.
	inside := ray.NewRay(vec.NewPoint3(0, 0, 0), vec.NewVec3(1, 0, 0))
	overlap, hit := box.Hit(inside, interval)
	if !hit || overlap.Min() != 0 || overlap.Max() != 1 {
		t.Fatalf("inside-ray overlap = %v hit=%v, want [0,1]", overlap, hit)
	}

	// Ray moving away from the box behind its origin: miss.
	away := ray.NewRay(vec.NewPoint3(5, 0, 0), vec.NewVec3(1, 0, 0))
	if _, hit := box.Hit(away, interval); hit {
		t.Fatal("ray moving away must miss")
	}

	// NaN direction component: miss, no panic.
	nanRay := ray.NewRay(vec.NewPoint3(0, 0, 0), vec.NewVec3(math.NaN(), 1, 1))
	if _, hit := box.Hit(nanRay, interval); hit {
		t.Fatal("NaN direction must miss")
	}

	// Empty box never hits.
	if _, hit := EmptyAABB().Hit(parallelHit, interval); hit {
		t.Fatal("empty box must never hit")
	}
}

func TestBVHDeterministicBuild(t *testing.T) {
	world := buildRandomScene(t, 11)
	first, err := world.BuildBVH()
	if err != nil {
		t.Fatalf("unexpected build error: %v", err)
	}
	second, err := world.BuildBVH()
	if err != nil {
		t.Fatalf("unexpected build error: %v", err)
	}
	if !sameTree(first, second) {
		t.Fatal("building the BVH twice over the same scene produced different trees")
	}
}

func sameTree(a, b *BVHNode) bool {
	if a == nil || b == nil {
		return a == b
	}
	if a.box != b.box {
		return false
	}
	aLeaf, aIsLeaf := a.left.(*BVHNode)
	bLeaf, bIsLeaf := b.left.(*BVHNode)
	if aIsLeaf != bIsLeaf {
		return false
	}
	if aIsLeaf && !sameTree(aLeaf, bLeaf) {
		return false
	}
	aRight, aRightLeaf := a.right.(*BVHNode)
	bRight, bRightLeaf := b.right.(*BVHNode)
	if aRightLeaf != bRightLeaf {
		return false
	}
	if aRightLeaf && !sameTree(aRight, bRight) {
		return false
	}
	return true
}

// BenchmarkTraversal measures ray traversal cost for the linear list and the
// BVH on a sparse (~15 objects) and a dense (~1900 objects) fixed-seed scene.
func BenchmarkTraversal(b *testing.B) {
	scenes := []struct {
		name string
		grid int
	}{
		{name: "sparse", grid: 2},
		{name: "dense", grid: 22},
	}

	for _, scene := range scenes {
		world := buildRandomScene(b, scene.grid)
		bvh, err := world.BuildBVH()
		if err != nil {
			b.Fatal(err)
		}
		rng := rand.New(rand.NewSource(123456))
		rays := fixedRays(rng, 1000)
		hitInterval := utils.NewInterval(0.001, math.Inf(1))

		b.Run(scene.name+"/list", func(b *testing.B) {
			for range b.N {
				for _, r := range rays {
					world.Hit(r, hitInterval)
				}
			}
		})
		b.Run(scene.name+"/bvh", func(b *testing.B) {
			for range b.N {
				for _, r := range rays {
					bvh.Hit(r, hitInterval)
				}
			}
		})
	}
}
