package main

import (
	"flag"
	"fmt"

	"github.com/davidelettieri/raytracing-one-weekend-go/camera"
	"github.com/davidelettieri/raytracing-one-weekend-go/hittable"
	"github.com/davidelettieri/raytracing-one-weekend-go/utils"
	"github.com/davidelettieri/raytracing-one-weekend-go/vec"
)

func main() {
	accel := flag.String("accel", "bvh", "scene traversal mode: bvh (bounding volume hierarchy) or list (original linear traversal)")
	seed := flag.Int64("seed", 20240101, "fixed random seed for scene placement and sampling; same seed gives reproducible output")
	renderMode := flag.String("render", "goroutines", "renderer: goroutines (concurrent scanlines) or sync (deterministic single-thread order)")
	grid := flag.Int("grid", 11, "half-extent of the random-sphere grid: a grid of (2*grid)^2 candidate cells is generated (11 reproduces the original scene)")
	imageWidth := flag.Int("width", 1200, "rendered image width in pixels")
	samplesPerPixel := flag.Int("samples", 100, "samples per pixel")
	flag.Parse()

	utils.Seed(*seed)

	world := buildWorld(*grid)

	var scene hittable.Hittable = world
	switch *accel {
	case "bvh":
		bvh, err := world.BuildBVH()
		if err != nil {
			panic(fmt.Sprintf("cannot build BVH: %v", err))
		}
		scene = bvh
	case "list":
		// Keep the original linear per-ray traversal over every object.
	default:
		panic(fmt.Sprintf("unknown accel mode %q: use bvh or list", *accel))
	}

	lookFrom := vec.NewPoint3(13, 2, 2)
	lookTo := vec.NewPoint3(0, 0, 0)
	upDirection := vec.NewVec3(0, 1, 0)
	cam := camera.NewCamera(16.0/9.0, *imageWidth, *samplesPerPixel, 50, 20, lookFrom, lookTo, upDirection, 0.6, 10.0)

	switch *renderMode {
	case "goroutines":
		cam.RenderWithGoRoutines(scene)
	case "sync":
		cam.Render(scene)
	default:
		panic(fmt.Sprintf("unknown render mode %q: use goroutines or sync", *renderMode))
	}
}

func buildWorld(grid int) hittable.HittableList {
	world := hittable.NewHittableList()

	refPoint := vec.NewPoint3(4, 0.2, 0)
	groundMaterial := hittable.NewLambertian(vec.NewColor(0.5, 0.5, 0.5))
	world.Add(hittable.NewSphere(vec.NewPoint3(0, -1000, 0), 1000, groundMaterial))

	for a := -float64(grid); a < float64(grid); a++ {
		for b := -float64(grid); b < float64(grid); b++ {
			chooseMat := utils.RandomFloat64()
			center := vec.NewPoint3(a+0.9*utils.RandomFloat64(), 0.2, b+0.9*utils.RandomFloat64())

			if center.Subtract(refPoint).Length() > 0.9 {
				if chooseMat < 0.8 {
					// diffuse
					albedo := vec.ComponentsMultiply(vec.Random(), vec.Random())
					sphereMaterial := hittable.NewLambertian(albedo)
					world.Add(hittable.NewSphere(center, 0.2, sphereMaterial))
				} else if chooseMat < 0.95 {
					albedo := vec.RandomInInterval(0.5, 1)
					fuzz := utils.RandomFloat64InInterval(0, 0.5)
					sphereMaterial := hittable.NewMetal(albedo, fuzz)
					world.Add(hittable.NewSphere(center, 0.2, sphereMaterial))
				} else {
					sphereMaterial := hittable.NewDielectric(1.5)
					world.Add(hittable.NewSphere(center, 0.2, sphereMaterial))
				}
			}
		}
	}

	material1 := hittable.NewDielectric(1.5)
	world.Add(hittable.NewSphere(vec.NewPoint3(0, 1, 0), 1.0, material1))

	material2 := hittable.NewLambertian(vec.NewColor(0.4, 0.2, 0.1))
	world.Add(hittable.NewSphere(vec.NewPoint3(-4, 1, 0), 1.0, material2))

	material3 := hittable.NewMetal(vec.NewColor(0.7, 0.6, 0.5), 0.0)
	world.Add(hittable.NewSphere(vec.NewPoint3(4, 1, 0), 1.0, material3))

	return world
}
