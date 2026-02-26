// Chapter 6.3 — Boolean Difference tests.
//
// Iteration deliverable: box minus sphere = box with spherical cavity; mass verified.
// Tests verify:
//   1. Box [0,3]^3 minus sphere (center inside box, radius 0.5):
//      volume ≈ V_box − V_sphere
//   2. Center of the subtracted sphere is classified kOutside the result
//   3. A corner of the box (outside sphere) is classified kInside the result
//   4. Difference of a box from itself has near-zero volume (kOutside all points)

#include "GkTest.h"
#include "gk/boolean/BooleanOps.h"
#include "gk/brep/BRepQuery.h"
#include "gk/builders/PrimitiveBuilders.h"
#include <cmath>

using namespace gk;

static constexpr double kPi  = 3.14159265358979323846;
static constexpr double kTol = 0.05;   // 5 % volume tolerance

// =============================================================================
// Box minus sphere
// =============================================================================

GK_TEST(Boolean6_3, BoxMinusSphere_VolumeIsCorrect)
{
    // Box [0,3]^3, volume = 27
    // Sphere at (1.5, 1.5, 1.5), radius = 0.5, volume = 4π/3 × 0.125 ≈ 0.524
    auto box    = makeBox(Vec3{0, 0, 0}, Vec3{3, 3, 3});
    auto sphere = makeSphere(Vec3{1.5, 1.5, 1.5}, 0.5);
    auto result = BooleanDifference(box, sphere);

    double vBox    = 27.0;
    double vSphere = 4.0 / 3.0 * kPi * 0.5 * 0.5 * 0.5;
    double expected = vBox - vSphere;

    auto mp = BRepQuery::computeMassProperties(*result, 32);
    EXPECT_NEAR(mp.volume, expected, kTol);
}

GK_TEST(Boolean6_3, BoxMinusSphere_SphereCenter_IsOutside)
{
    auto box    = makeBox(Vec3{0, 0, 0}, Vec3{3, 3, 3});
    auto sphere = makeSphere(Vec3{1.5, 1.5, 1.5}, 0.5);
    auto result = BooleanDifference(box, sphere);

    // The sphere centre is in the cavity — must be outside the solid
    EXPECT_EQ(BRepQuery::classifyPoint(*result, Vec3{1.5, 1.5, 1.5}),
              PointLocation::kOutside);
}

GK_TEST(Boolean6_3, BoxMinusSphere_BoxCorner_IsInside)
{
    auto box    = makeBox(Vec3{0, 0, 0}, Vec3{3, 3, 3});
    auto sphere = makeSphere(Vec3{1.5, 1.5, 1.5}, 0.5);
    auto result = BooleanDifference(box, sphere);

    // A corner of the box far from the sphere is inside the solid
    EXPECT_EQ(BRepQuery::classifyPoint(*result, Vec3{0.1, 0.1, 0.1}),
              PointLocation::kInside);
}

// =============================================================================
// Box minus itself → zero volume
// =============================================================================

GK_TEST(Boolean6_3, BoxMinusSelf_IsZeroVolume)
{
    auto box    = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    auto result = BooleanDifference(box, box);

    EXPECT_TRUE(isZeroVolume(result, 0.01, 20));
}

// =============================================================================
// Box minus smaller nested box
// =============================================================================

GK_TEST(Boolean6_3, BoxMinusNestedBox_VolumeIsCorrect)
{
    // Outer box [0,4]^3 (V=64), inner box [1,3]^3 (V=8)
    auto outer = makeBox(Vec3{0, 0, 0}, Vec3{4, 4, 4});
    auto inner = makeBox(Vec3{1, 1, 1}, Vec3{3, 3, 3});
    auto result = BooleanDifference(outer, inner);

    auto mp = BRepQuery::computeMassProperties(*result, 24);
    EXPECT_NEAR(mp.volume, 64.0 - 8.0, 1.0);   // 56 ± 1
}

GK_TEST(Boolean6_3, BoxMinusNestedBox_InnerCenter_IsOutside)
{
    auto outer = makeBox(Vec3{0, 0, 0}, Vec3{4, 4, 4});
    auto inner = makeBox(Vec3{1, 1, 1}, Vec3{3, 3, 3});
    auto result = BooleanDifference(outer, inner);

    // Centre of the removed box is now a cavity
    EXPECT_EQ(BRepQuery::classifyPoint(*result, Vec3{2, 2, 2}),
              PointLocation::kOutside);
}
