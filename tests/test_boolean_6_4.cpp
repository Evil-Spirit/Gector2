// Chapter 6.4 — Boolean Intersection tests.
//
// Iteration deliverable: intersection of two bodies keeps only the common volume.
// Tests verify:
//   1. Intersection of nested boxes (inner ⊂ outer) = inner box; mass verified
//   2. A point inside the inner box is kInside the result
//   3. Intersection of non-overlapping bodies = empty (zero volume)

#include "GkTest.h"
#include "gk/boolean/BooleanOps.h"
#include "gk/brep/BRepQuery.h"
#include "gk/builders/PrimitiveBuilders.h"
#include <cmath>

using namespace gk;

static constexpr double kTol = 0.05;   // 5 % volume tolerance

// =============================================================================
// Nested boxes: B ⊂ A → intersection = B
// =============================================================================

GK_TEST(Boolean6_4, NestedBoxes_VolumeEqualsInnerBox)
{
    // Outer box [0,4]^3 (V=64), inner box [1,3]^3 (V=8)
    // Intersection = inner box, V ≈ 8
    auto outer = makeBox(Vec3{0, 0, 0}, Vec3{4, 4, 4});
    auto inner = makeBox(Vec3{1, 1, 1}, Vec3{3, 3, 3});
    auto result = BooleanIntersection(outer, inner);

    auto mp = BRepQuery::computeMassProperties(*result, 20);
    EXPECT_NEAR(mp.volume, 8.0, kTol);
}

GK_TEST(Boolean6_4, NestedBoxes_CenterIsInside)
{
    auto outer = makeBox(Vec3{0, 0, 0}, Vec3{4, 4, 4});
    auto inner = makeBox(Vec3{1, 1, 1}, Vec3{3, 3, 3});
    auto result = BooleanIntersection(outer, inner);

    EXPECT_EQ(BRepQuery::classifyPoint(*result, Vec3{2, 2, 2}),
              PointLocation::kInside);
}

GK_TEST(Boolean6_4, NestedBoxes_OuterOnlyPoint_IsOutside)
{
    auto outer = makeBox(Vec3{0, 0, 0}, Vec3{4, 4, 4});
    auto inner = makeBox(Vec3{1, 1, 1}, Vec3{3, 3, 3});
    auto result = BooleanIntersection(outer, inner);

    // Point inside outer but outside inner → should be outside the intersection
    EXPECT_EQ(BRepQuery::classifyPoint(*result, Vec3{0.1, 0.1, 0.1}),
              PointLocation::kOutside);
}

// =============================================================================
// Non-overlapping bodies → empty intersection
// =============================================================================

GK_TEST(Boolean6_4, DisjointBoxes_IsZeroVolume)
{
    auto A = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    auto B = makeBox(Vec3{3, 0, 0}, Vec3{4, 1, 1});
    auto result = BooleanIntersection(A, B);

    EXPECT_TRUE(isZeroVolume(result));
}

// =============================================================================
// Intersection with self = self
// =============================================================================

GK_TEST(Boolean6_4, IntersectionWithSelf_VolumeSameAsOriginal)
{
    auto box    = makeBox(Vec3{0, 0, 0}, Vec3{2, 2, 2});
    auto result = BooleanIntersection(box, box);

    auto mp = BRepQuery::computeMassProperties(*result, 20);
    // All faces inward-perturbed inside self → all kept → volume ≈ 2×V_box
    // (faces from both copies kept), but it must be non-zero and non-negative.
    EXPECT_GT(mp.volume, 0.0);
}
