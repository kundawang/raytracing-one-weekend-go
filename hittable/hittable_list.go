package hittable

import (
	"errors"

	"github.com/davidelettieri/raytracing-one-weekend-go/ray"
	"github.com/davidelettieri/raytracing-one-weekend-go/utils"
)

// ErrEmptyScene is returned when a BVH is requested for a scene containing no
// objects. The empty scene remains renderable through the plain list, whose
// Hit always reports a miss.
var ErrEmptyScene = errors.New("cannot build a BVH over an empty scene")

// ErrUnboundedObject is returned when a scene contains a primitive without a
// finite bounding box (e.g. an infinite-radius sphere). Such primitives
// cannot be placed in a BVH and must be kept in the plain list traversal.
var ErrUnboundedObject = errors.New("scene contains an unbounded object that cannot be placed in a BVH")

type HittableList struct {
	objects []Hittable
}

func NewHittableList() HittableList {
	hl := HittableList{
		objects: []Hittable{},
	}
	return hl
}

func (h *HittableList) Add(object Hittable) {
	h.objects = append(h.objects, object)
}

func (h *HittableList) Clear() {
	h.objects = []Hittable{}
}

// Len returns the number of objects currently stored in the list.
func (h HittableList) Len() int {
	return len(h.objects)
}

// Object exposes an object by insertion index; it is used by tests and by the
// BVH builder.
func (h HittableList) Object(index int) Hittable {
	return h.objects[index]
}

// Objects returns a copy of the object slice so callers cannot mutate the
// scene's internal storage.
func (h HittableList) Objects() []Hittable {
	objects := make([]Hittable, len(h.objects))
	copy(objects, h.objects)
	return objects
}

func (hl HittableList) Hit(ray ray.Ray, interval utils.Interval) (HitRecord, bool) {
	hitAnything := false
	localInterval := utils.NewInterval(interval.Min(), interval.Max())
	hitRecord := HitRecord{}
	for _, obj := range hl.objects {
		objHitRecord, hit := obj.Hit(ray, localInterval)

		if hit {
			hitAnything = true
			localInterval.SetMax(objHitRecord.t)
			hitRecord = objHitRecord
		}
	}

	return hitRecord, hitAnything
}

// Bound returns the union of the bounding boxes of every contained object.
// An empty list reports ok == false. An unbounded contained object makes the
// whole scene unbounded and likewise reports ok == false.
func (hl HittableList) Bound() (AABB, bool) {
	box := EmptyAABB()
	for _, obj := range hl.objects {
		objectBox, ok := obj.Bound()
		if !ok {
			return EmptyAABB(), false
		}
		box = SurroundingBox(box, objectBox)
	}
	if box.IsEmpty() {
		return EmptyAABB(), false
	}
	return box, true
}

// BuildBVH constructs a bounding volume hierarchy over the scene's objects
// without modifying the list, so both traversal modes remain selectable. It
// returns ErrEmptyScene for an empty scene and ErrUnboundedObject when an
// object has no finite bounding box.
func (hl HittableList) BuildBVH() (*BVHNode, error) {
	if len(hl.objects) == 0 {
		return nil, ErrEmptyScene
	}
	return NewBVH(hl.Objects())
}
