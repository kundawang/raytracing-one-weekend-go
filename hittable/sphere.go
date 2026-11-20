package hittable

import (
	"math"

	"github.com/davidelettieri/raytracing-one-weekend-go/ray"
	"github.com/davidelettieri/raytracing-one-weekend-go/utils"
	"github.com/davidelettieri/raytracing-one-weekend-go/vec"
)

// tinyRadiusThreshold separates ordinary spheres, which keep the original
// radius-inverse normal scaling for bit-identical results, from tiny spheres,
// whose scaled normals lose too much precision and are normalized instead.
const tinyRadiusThreshold = 1e-3

type Sphere struct {
	center        vec.Point3
	radius        float64
	material      Material
	radiusInv     float64
	radiusSquared float64
}

func (s Sphere) Hit(ray ray.Ray, interval utils.Interval) (HitRecord, bool) {
	oc := s.center.Subtract(ray.Origin())
	rayDirection := ray.Direction()
	a := rayDirection.LengthSquared()
	h := vec.Dot(rayDirection, oc)
	c := oc.LengthSquared() - s.radiusSquared

	discriminant := h*h - a*c
	if discriminant < 0 {
		return HitRecord{}, false
	}

	sqrtd := math.Sqrt(discriminant)

	root := (h - sqrtd) / a

	if !interval.Surrounds(root) {
		root = (h + sqrtd) / a
		if !interval.Surrounds(root) {
			return HitRecord{}, false
		}
	}

	p := ray.At(root)

	var outwardNormal vec.Vec3
	switch {
	case s.radius == 0:
		// Point primitive hit exactly on its centre: p-center is the zero
		// vector, so the usual radius-inverse scaling would multiply by Inf
		// and produce a NaN normal that panics SetFaceNormal. Define the
		// outward normal as the unit vector facing the incoming ray instead.
		outwardNormal = rayDirection.Negate().Unit()
	case s.radius < tinyRadiusThreshold:
		// For very small radii the cancellation error in ray.At(t) dominates
		// the offset vector: radius-inverse scaling produces a non-unit normal
		// that fails SetFaceNormal's check. Explicit normalization is stable
		// while the offset keeps a direction. When the radius is so small that
		// the offset underflows to zero entirely, fall back to the unit vector
		// facing the incoming ray (same convention as a point-primitive hit).
		offset := p.Subtract(s.center)
		if offset.NearZero() {
			outwardNormal = rayDirection.Negate().Unit()
		} else {
			outwardNormal = offset.Unit()
		}
	default:
		// Keep the original radius-inverse scaling so hit records stay bit for
		// bit identical to the pre-BVH implementation at ordinary scales.
		outwardNormal = p.Subtract(s.center).Multiply(s.radiusInv)
	}

	rec := HitRecord{
		t:        root,
		p:        p,
		material: s.material,
	}

	rec.SetFaceNormal(ray, outwardNormal)

	return rec, true
}

func NewSphere(center vec.Point3, radius float64, material Material) Sphere {
	// A negative radius is physically meaningless: clamp it to zero (a point
	// primitive) instead of storing it, which would invert radiusInv's sign
	// and flip every outward normal. radiusSquared stays in sync with the
	// clamped value so Hit's quadratic uses the same effective radius.
	radius = math.Max(0, radius)
	return Sphere{
		center:        center,
		radius:        radius,
		material:      material,
		radiusInv:     1 / radius,
		radiusSquared: radius * radius,
	}
}

// Bound returns the axis-aligned box enclosing the sphere. A zero-radius
// sphere is a point primitive: NewAABB pads its zero-extent box so it stays
// intersectable. Infinite or NaN radii are unbounded/non-finite primitives
// and report ok == false so callers (e.g. the BVH builder) can reject them.
func (s Sphere) Bound() (AABB, bool) {
	if math.IsInf(s.radius, 0) || math.IsNaN(s.radius) {
		return EmptyAABB(), false
	}

	offset := vec.NewVec3(s.radius, s.radius, s.radius)
	return NewAABB(s.center.Subtract(offset), s.center.Add(offset)), true
}
