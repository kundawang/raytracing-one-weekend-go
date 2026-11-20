package camera_test

import (
	"math/rand"
	"testing"

	"github.com/davidelettieri/raytracing-one-weekend-go/camera"
	"github.com/davidelettieri/raytracing-one-weekend-go/hittable"
	"github.com/davidelettieri/raytracing-one-weekend-go/utils"
	"github.com/davidelettieri/raytracing-one-weekend-go/vec"
)

// buildSeededWorld mirrors main.go's multi-sphere scene but draws from a
// local fixed-seed RNG passed in by the caller, so both traversal modes render
// an identical scene.
func buildSeededWorld(rng *rand.Rand) hittable.HittableList {
	randomFloat := func() float64 { return rng.Float64() }

	world := hittable.NewHittableList()
	refPoint := vec.NewPoint3(4, 0.2, 0)
	world.Add(hittable.NewSphere(vec.NewPoint3(0, -1000, 0), 1000, hittable.NewLambertian(vec.NewColor(0.5, 0.5, 0.5))))

	for a := -11.0; a < 11; a++ {
		for b := -11.0; b < 11; b++ {
			chooseMat := randomFloat()
			center := vec.NewPoint3(a+0.9*randomFloat(), 0.2, b+0.9*randomFloat())
			if center.Subtract(refPoint).Length() <= 0.9 {
				continue
			}
			switch {
			case chooseMat < 0.8:
				albedo := vec.NewVec3(randomFloat(), randomFloat(), randomFloat())
				world.Add(hittable.NewSphere(center, 0.2, hittable.NewLambertian(albedo)))
			case chooseMat < 0.95:
				albedo := vec.NewVec3(0.5+0.5*randomFloat(), 0.5+0.5*randomFloat(), 0.5+0.5*randomFloat())
				world.Add(hittable.NewSphere(center, 0.2, hittable.NewMetal(albedo, 0.5*randomFloat())))
			default:
				world.Add(hittable.NewSphere(center, 0.2, hittable.NewDielectric(1.5)))
			}
		}
	}

	world.Add(hittable.NewSphere(vec.NewPoint3(0, 1, 0), 1.0, hittable.NewDielectric(1.5)))
	world.Add(hittable.NewSphere(vec.NewPoint3(-4, 1, 0), 1.0, hittable.NewLambertian(vec.NewColor(0.4, 0.2, 0.1))))
	world.Add(hittable.NewSphere(vec.NewPoint3(4, 1, 0), 1.0, hittable.NewMetal(vec.NewColor(0.7, 0.6, 0.5), 0.0)))
	return world
}

// TestRenderPixelIdenticalListAndBVH renders the same fixed-seed multi-sphere
// scene twice (linear list and BVH) using the deterministic single-threaded
// renderer and asserts every accumulated pixel color is bit-identical.
// Scattering, occlusion and the RNG sequence are untouched, so any difference
// can only come from the traversal returning a different nearest hit.
func TestRenderPixelIdenticalListAndBVH(t *testing.T) {
	const (
		seed     int64 = 20240101
		width          = 160
		samples        = 8
		maxDepth       = 20
	)

	world := buildSeededWorld(rand.New(rand.NewSource(seed)))
	bvh, err := world.BuildBVH()
	if err != nil {
		t.Fatalf("BVH build failed: %v", err)
	}

	cam := camera.NewCamera(
		16.0/9.0, width, samples, maxDepth, 20,
		vec.NewPoint3(13, 2, 2), vec.NewPoint3(0, 0, 0), vec.NewVec3(0, 1, 0),
		0.6, 10.0,
	)

	// The global RNG drives camera sampling and material scattering. Seed it
	// identically before each render so both runs consume the same sequence.
	utils.Seed(seed)
	listPixels := cam.RenderToPixels(world)

	utils.Seed(seed)
	bvhPixels := cam.RenderToPixels(bvh)

	if len(listPixels) != len(bvhPixels) {
		t.Fatalf("pixel count mismatch: list=%d bvh=%d", len(listPixels), len(bvhPixels))
	}

	differences := 0
	for i := range listPixels {
		if listPixels[i] != bvhPixels[i] {
			differences++
			if differences <= 5 {
				t.Errorf("pixel %d differs: list=%v bvh=%v", i, listPixels[i], bvhPixels[i])
			}
		}
	}
	if differences != 0 {
		t.Fatalf("%d/%d pixels differ between list and BVH traversal", differences, len(listPixels))
	}
	t.Logf("all %d pixels bit-identical between list and BVH (%d objects, %d samples/pixel)",
		len(listPixels), world.Len(), samples)
}
