package hittable

import (
	"sort"

	"github.com/davidelettieri/raytracing-one-weekend-go/ray"
	"github.com/davidelettieri/raytracing-one-weekend-go/utils"
)

// BVHNode is a node of a bounding volume hierarchy. Leaves keep one or two
// primitives directly; internal nodes own exactly two children.
type BVHNode struct {
	left  Hittable
	right Hittable // nil for single-object leaves
	box   AABB
}

// NewBVH builds a hierarchy over a copy of objects. The build is fully
// deterministic: the same input scene always yields the same tree.
//
// Split rule (applied at every internal node):
//  1. Compute the bounding box of every object's centroid and choose its
//     longest axis (ties resolved X, then Y, then Z).
//  2. Stably sort the objects by their centroid coordinate on that axis.
//     Stable sort preserves insertion order for coincident objects, so the
//     resulting tree (and equal-distance hit tie-breaking) is reproducible.
//  3. Split the sorted slice in half. A node of two objects becomes a leaf
//     holding one object per child; a single object becomes a leaf.
//
// Because objects with identical centroids compare equal, all-coincident and
// pathologically overlapping scenes simply halve by insertion order instead
// of degenerating into a linear chain.
func NewBVH(objects []Hittable) (*BVHNode, error) {
	objectsCopy := make([]Hittable, len(objects))
	copy(objectsCopy, objects)
	return buildBVH(objectsCopy)
}

func buildBVH(objects []Hittable) (*BVHNode, error) {
	node := &BVHNode{}

	switch len(objects) {
	case 1:
		box, ok := objects[0].Bound()
		if !ok {
			return nil, ErrUnboundedObject
		}
		node.left = objects[0]
		node.box = box
		return node, nil

	case 2:
		leftBox, leftOK := objects[0].Bound()
		rightBox, rightOK := objects[1].Bound()
		if !leftOK || !rightOK {
			return nil, ErrUnboundedObject
		}
		node.left = objects[0]
		node.right = objects[1]
		node.box = SurroundingBox(leftBox, rightBox)
		return node, nil
	}

	axis := longestCentroidAxis(objects)
	sort.SliceStable(objects, func(i, j int) bool {
		return centroidLess(objects[i], objects[j], axis)
	})

	middle := len(objects) / 2
	var err error
	node.left, err = buildBVH(objects[:middle])
	if err != nil {
		return nil, err
	}
	node.right, err = buildBVH(objects[middle:])
	if err != nil {
		return nil, err
	}

	leftBox, _ := node.left.Bound()
	rightBox, _ := node.right.Bound()
	node.box = SurroundingBox(leftBox, rightBox)
	return node, nil
}

// centroidLess compares objects by centroid coordinate on axis, falling back
// to the remaining axes (and finally false) so the ordering is a total order
// even for coincident centroids, where stable sort then keeps insertion order.
func centroidLess(a, b Hittable, axis int) bool {
	boxA, okA := a.Bound()
	boxB, okB := b.Bound()
	if !okA || !okB {
		return false
	}
	centerA := boxA.Center()
	centerB := boxB.Center()
	for offset := 0; offset < 3; offset++ {
		currentAxis := (axis + offset) % 3
		va, vb := centerA.Index(currentAxis), centerB.Index(currentAxis)
		if va != vb {
			return va < vb
		}
	}
	return false
}

// longestCentroidAxis picks the axis with the largest extent of the bounding
// box enclosing all object centroids. Ties prefer X, then Y, then Z.
func longestCentroidAxis(objects []Hittable) int {
	centroidBox := EmptyAABB()
	for _, obj := range objects {
		box, ok := obj.Bound()
		if !ok {
			continue
		}
		centroid := box.Center()
		centroidBox = SurroundingBox(centroidBox, NewAABB(centroid, centroid))
	}

	extentX := centroidBox.Max().X() - centroidBox.Min().X()
	extentY := centroidBox.Max().Y() - centroidBox.Min().Y()
	extentZ := centroidBox.Max().Z() - centroidBox.Min().Z()

	switch {
	case extentX >= extentY && extentX >= extentZ:
		return 0
	case extentY >= extentZ:
		return 1
	default:
		return 2
	}
}

// Hit traverses the hierarchy, shrinking the ray interval as nearer hits are
// found. The interval-shrinking scheme is the same one used by
// HittableList.Hit: regardless of traversal order, the primitive with the
// smallest t inside the interval wins, and ties (equal t) keep the first
// object in insertion order because leaves preserve that order and strict
// interval shrinking never overwrites an equal-t record.
func (node *BVHNode) Hit(r ray.Ray, rayT utils.Interval) (HitRecord, bool) {
	if _, ok := node.box.Hit(r, rayT); !ok {
		return HitRecord{}, false
	}

	record, hitLeft := node.left.Hit(r, rayT)
	if hitLeft {
		rayT.SetMax(record.t)
	}

	if node.right != nil {
		rightRecord, hitRight := node.right.Hit(r, rayT)
		if hitRight {
			return rightRecord, true
		}
	}

	return record, hitLeft
}

// Bound exposes the node's enclosing box so subtrees satisfy Hittable.
func (node *BVHNode) Bound() (AABB, bool) {
	return node.box, true
}
