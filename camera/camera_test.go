package camera_test

import (
	"math/rand"
	"testing"

	"github.com/davidelettieri/raytracing-one-weekend-go/camera"
	"github.com/davidelettieri/raytracing-one-weekend-go/hittable"
	"github.com/davidelettieri/raytracing-one-weekend-go/utils"
	"github.com/davidelettieri/raytracing-one-weekend-go/vec"
)

func BenchmarkRender(b *testing.B) {
	world := hittable.NewHittableList()

	materialGround := hittable.NewLambertian(vec.NewColor(0.8, 0.8, 0.0))
	materialCenter := hittable.NewLambertian(vec.NewColor(0.1, 0.2, 0.5))
	materialLeft := hittable.NewDielectric(1.5)
	materialBubble := hittable.NewDielectric(1.0 / 1.5)
	materialRight := hittable.NewMetal(vec.NewColor(0.8, 0.6, 0.2), 1)

	world.Add(hittable.NewSphere(vec.NewPoint3(0, -100.5, -1), 100, materialGround))
	world.Add(hittable.NewSphere(vec.NewPoint3(0, 0, -1.2), 0.5, materialCenter))
	world.Add(hittable.NewSphere(vec.NewPoint3(-1, 0, -1), 0.5, materialLeft))
	world.Add(hittable.NewSphere(vec.NewPoint3(-1, 0, -1), 0.4, materialBubble))
	world.Add(hittable.NewSphere(vec.NewPoint3(1, 0, -1), 0.5, materialRight))
	lookFrom := vec.NewPoint3(-2, 2, 1)
	lookTo := vec.NewPoint3(0, 0, -1)
	upDirection := vec.NewVec3(0, 1, 0)
	cam := camera.NewCamera(16.0/9.0, 400, 100, 50, 20, lookFrom, lookTo, upDirection, 10, 3.4)
	cam.Render(world)
}

func BenchmarkRenderWithGoRoutines(b *testing.B) {
	world := hittable.NewHittableList()

	materialGround := hittable.NewLambertian(vec.NewColor(0.8, 0.8, 0.0))
	materialCenter := hittable.NewLambertian(vec.NewColor(0.1, 0.2, 0.5))
	materialLeft := hittable.NewDielectric(1.5)
	materialBubble := hittable.NewDielectric(1.0 / 1.5)
	materialRight := hittable.NewMetal(vec.NewColor(0.8, 0.6, 0.2), 1)

	world.Add(hittable.NewSphere(vec.NewPoint3(0, -100.5, -1), 100, materialGround))
	world.Add(hittable.NewSphere(vec.NewPoint3(0, 0, -1.2), 0.5, materialCenter))
	world.Add(hittable.NewSphere(vec.NewPoint3(-1, 0, -1), 0.5, materialLeft))
	world.Add(hittable.NewSphere(vec.NewPoint3(-1, 0, -1), 0.4, materialBubble))
	world.Add(hittable.NewSphere(vec.NewPoint3(1, 0, -1), 0.5, materialRight))
	lookFrom := vec.NewPoint3(-2, 2, 1)
	lookTo := vec.NewPoint3(0, 0, -1)
	upDirection := vec.NewVec3(0, 1, 0)
	cam := camera.NewCamera(16.0/9.0, 400, 100, 50, 20, lookFrom, lookTo, upDirection, 10, 3.4)
	cam.RenderWithGoRoutines(world)
}

// BenchmarkSceneRenderListVsBVH renders the fixed-seed multi-sphere scene
// in memory (no stdout) under both traversal modes, giving an end-to-end,
// reproducible comparison that includes camera sampling and scattering.
func BenchmarkSceneRenderListVsBVH(b *testing.B) {
	const seed int64 = 20240101
	world := buildSeededWorld(rand.New(rand.NewSource(seed)))
	bvh, err := world.BuildBVH()
	if err != nil {
		b.Fatal(err)
	}

	cam := camera.NewCamera(
		16.0/9.0, 200, 16, 20, 20,
		vec.NewPoint3(13, 2, 2), vec.NewPoint3(0, 0, 0), vec.NewVec3(0, 1, 0),
		0.6, 10.0,
	)

	scenes := map[string]hittable.Hittable{"list": world, "bvh": bvh}
	for name, scene := range scenes {
		b.Run(name, func(b *testing.B) {
			for range b.N {
				utils.Seed(seed)
				cam.RenderToPixels(scene)
			}
		})
	}
}
