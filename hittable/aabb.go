package hittable

import (
	"math"

	"github.com/davidelettieri/raytracing-one-weekend-go/ray"
	"github.com/davidelettieri/raytracing-one-weekend-go/utils"
	"github.com/davidelettieri/raytracing-one-weekend-go/vec"
)

// aabbEpsilon pads boxes with zero extent along an axis (e.g. point or
// zero-radius spheres) so that every stored box is a closed, non-degenerate
// volume and the slab test never divides by zero.
const aabbEpsilon = 1e-8

// AABB is an axis-aligned bounding box stored as two corner points.
type AABB struct {
	minimum vec.Point3
	maximum vec.Point3
}

// NewAABB builds a box from two corner points, padding every zero-extent
// axis by aabbEpsilon so the box stays non-degenerate.
func NewAABB(a, b vec.Point3) AABB {
	pad := func(lo, hi float64) (float64, float64) {
		if hi-lo < aabbEpsilon {
			mid := (lo + hi) / 2
			return mid - aabbEpsilon/2, mid + aabbEpsilon/2
		}
		return lo, hi
	}

	x0, x1 := pad(math.Min(a.X(), b.X()), math.Max(a.X(), b.X()))
	y0, y1 := pad(math.Min(a.Y(), b.Y()), math.Max(a.Y(), b.Y()))
	z0, z1 := pad(math.Min(a.Z(), b.Z()), math.Max(a.Z(), b.Z()))

	return AABB{
		minimum: vec.NewPoint3(x0, y0, z0),
		maximum: vec.NewPoint3(x1, y1, z1),
	}
}

// EmptyAABB returns a box for which IsEmpty is true (min > max on every axis).
func EmptyAABB() AABB {
	inf := math.Inf(1)
	return AABB{
		minimum: vec.NewPoint3(inf, inf, inf),
		maximum: vec.NewPoint3(-inf, -inf, -inf),
	}
}

// IsEmpty reports whether the box contains no points.
func (box AABB) IsEmpty() bool {
	return box.minimum.X() > box.maximum.X() ||
		box.minimum.Y() > box.maximum.Y() ||
		box.minimum.Z() > box.maximum.Z()
}

func (box AABB) Min() vec.Point3 { return box.minimum }
func (box AABB) Max() vec.Point3 { return box.maximum }

// Center returns the geometric centre of the box, used by the BVH split rule.
func (box AABB) Center() vec.Point3 {
	return box.minimum.Add(box.maximum).Multiply(0.5)
}

// SurroundingBox returns the smallest (epsilon-padded) box enclosing both
// inputs. An empty operand is ignored. The union of two empty boxes is empty.
func SurroundingBox(box0, box1 AABB) AABB {
	if box0.IsEmpty() {
		return box1
	}
	if box1.IsEmpty() {
		return box0
	}

	small := vec.NewPoint3(
		math.Min(box0.minimum.X(), box1.minimum.X()),
		math.Min(box0.minimum.Y(), box1.minimum.Y()),
		math.Min(box0.minimum.Z(), box1.minimum.Z()),
	)
	big := vec.NewPoint3(
		math.Max(box0.maximum.X(), box1.maximum.X()),
		math.Max(box0.maximum.Y(), box1.maximum.Y()),
		math.Max(box0.maximum.Z(), box1.maximum.Z()),
	)
	return NewAABB(small, big)
}

// Hit applies the slab (inverse-direction) intersection test. It returns the
// overlap interval between the ray segment ray.T in rayT and the box.
//
// The inverse-direction formulation handles rays parallel to a box face
// (zero direction component): division yields +/-Inf and the comparisons
// stay finite and correct. A ray with a NaN direction component never hits.
// Rays originating inside the box intersect with tMin = -Inf, which is
// compatible with the open-interval (Surrounds) semantics used elsewhere.
func (box AABB) Hit(r ray.Ray, rayT utils.Interval) (utils.Interval, bool) {
	if box.IsEmpty() {
		return rayT, false
	}

	tMin := rayT.Min()
	tMax := rayT.Max()

	origin := r.Origin()
	direction := r.Direction()

	for axis := 0; axis < 3; axis++ {
		originComponent := origin.Index(axis)
		directionComponent := direction.Index(axis)

		if directionComponent == 0 {
			// Parallel to the slab: the ray can only hit if its origin lies
			// inside the slab extent; the interval itself is unchanged.
			if originComponent < box.minimum.Index(axis) || originComponent > box.maximum.Index(axis) {
				return rayT, false
			}
			continue
		}

		if math.IsNaN(directionComponent) {
			return rayT, false
		}

		invDirection := 1.0 / directionComponent
		t0 := (box.minimum.Index(axis) - originComponent) * invDirection
		t1 := (box.maximum.Index(axis) - originComponent) * invDirection
		if t0 > t1 {
			t0, t1 = t1, t0
		}

		// Plain comparisons instead of math.Min/Max: NaN would otherwise make
		// the bounds NaN and silently pass every later comparison.
		if t0 > tMin {
			tMin = t0
		}
		if t1 < tMax {
			tMax = t1
		}

		if tMax <= tMin {
			return rayT, false
		}
	}

	return utils.NewInterval(tMin, tMax), true
}
