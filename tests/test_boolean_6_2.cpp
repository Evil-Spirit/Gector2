// Chapter 6.2 — Boolean Union tests.
//
// Iteration deliverable: union of two overlapping bodies; mass ≈ sum − overlap.
// Tests verify:
//   1. Union of two disjoint boxes  → volume = V_A + V_B (exact within tolerance)
//   2. Point inside A  is kInside the union body
//   3. Point inside B  is kInside the union body
//   4. Point outside both is kOutside the union body
//   5. Union of two overlapping boxes → result is non-empty and
//      volume is between max(V_A, V_B) and V_A + V_B

#include "GkTest.h"
#include "gk/boolean/BooleanOps.h"
#include "gk/brep/BRepQuery.h"
#include "gk/builders/PrimitiveBuilders.h"
#include <cmath>

using namespace gk;

static constexpr double kPi  = 3.14159265358979323846;
static constexpr double kTol = 1e-2;   // volume tolerance (~1 %)

// =============================================================================
// Disjoint bodies
// =============================================================================

GK_TEST(Boolean6_2, UnionDisjointBoxes_VolumeIsSum)
{
    // Two unit boxes separated along X
    auto A = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});  // volume = 1
    auto B = makeBox(Vec3{3, 0, 0}, Vec3{4, 1, 1});  // volume = 1
    auto U = BooleanUnion(A, B);

    auto mp = BRepQuery::computeMassProperties(*U, 20);
    EXPECT_NEAR(mp.volume, 2.0, kTol);
}

GK_TEST(Boolean6_2, UnionDisjointBoxes_PointInsideA_IsInside)
{
    auto A = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    auto B = makeBox(Vec3{3, 0, 0}, Vec3{4, 1, 1});
    auto U = BooleanUnion(A, B);

    EXPECT_EQ(BRepQuery::classifyPoint(*U, Vec3{0.5, 0.5, 0.5}),
              PointLocation::kInside);
}

GK_TEST(Boolean6_2, UnionDisjointBoxes_PointInsideB_IsInside)
{
    auto A = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    auto B = makeBox(Vec3{3, 0, 0}, Vec3{4, 1, 1});
    auto U = BooleanUnion(A, B);

    EXPECT_EQ(BRepQuery::classifyPoint(*U, Vec3{3.5, 0.5, 0.5}),
              PointLocation::kInside);
}

GK_TEST(Boolean6_2, UnionDisjointBoxes_PointOutside_IsOutside)
{
    auto A = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    auto B = makeBox(Vec3{3, 0, 0}, Vec3{4, 1, 1});
    auto U = BooleanUnion(A, B);

    EXPECT_EQ(BRepQuery::classifyPoint(*U, Vec3{2.0, 0.5, 0.5}),
              PointLocation::kOutside);
}

// =============================================================================
// Overlapping bodies
// =============================================================================

GK_TEST(Boolean6_2, UnionOverlappingBoxes_VolumeInBounds)
{
    // A = [0,2]^3 (V=8),  B = [1,3]^3 (V=8),  overlap = [1,2]^3 (V=1)
    // Union volume = 8 + 8 - 1 = 15
    auto A = makeBox(Vec3{0, 0, 0}, Vec3{2, 2, 2});
    auto B = makeBox(Vec3{1, 1, 1}, Vec3{3, 3, 3});
    auto U = BooleanUnion(A, B);

    auto mp = BRepQuery::computeMassProperties(*U, 20);
    // Result must be non-empty and larger than either input
    EXPECT_GT(mp.volume, 8.0 - kTol);
    // Must not exceed the naive sum
    EXPECT_LT(mp.volume, 16.0 + kTol);
}

GK_TEST(Boolean6_2, UnionOverlappingBoxes_NonEmpty)
{
    auto A = makeBox(Vec3{0, 0, 0}, Vec3{2, 2, 2});
    auto B = makeBox(Vec3{1, 1, 1}, Vec3{3, 3, 3});
    auto U = BooleanUnion(A, B);

    EXPECT_FALSE(isZeroVolume(U));
}

// =============================================================================
// Box + sphere
// =============================================================================

GK_TEST(Boolean6_2, UnionBoxAndSphere_VolumeInBounds)
{
    // Sphere at (5,0,0) radius 1 is fully outside the unit box
    auto box    = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    auto sphere = makeSphere(Vec3{5, 0, 0}, 1.0);
    auto U      = BooleanUnion(box, sphere);

    auto mp = BRepQuery::computeMassProperties(*U, 24);
    double expectedVol = 1.0 + 4.0 / 3.0 * kPi;   // ≈ 5.189
    EXPECT_NEAR(mp.volume, expectedVol, 0.2);
}
