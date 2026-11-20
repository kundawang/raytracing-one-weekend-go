package utils

import (
	"math/rand"
	"sync"
)

const Pi = 3.1415926535897932385

// rngMu guards rng. The renderer calls RandomFloat64 from one goroutine per
// scanline, so the source needs external synchronization.
var (
	rngMu sync.Mutex
	rng   = rand.New(rand.NewSource(rand.Int63()))
)

// Seed resets the global random source to a deterministic state. With the
// same seed, the sequence of RandomFloat64 values (and therefore scene
// placement, camera sampling and material scattering) is reproducible, which
// is what makes the list-vs-BVH pixel comparison and benchmarks repeatable.
func Seed(value int64) {
	rngMu.Lock()
	defer rngMu.Unlock()
	rng = rand.New(rand.NewSource(value))
}

func DegreesToRadians(degress float64) float64 {
	return degress * Pi / 180.0
}

func RandomFloat64() float64 {
	rngMu.Lock()
	defer rngMu.Unlock()
	return rng.Float64()
}

func RandomFloat64InInterval(min, max float64) float64 {
	return min + (max-min)*RandomFloat64()
}
