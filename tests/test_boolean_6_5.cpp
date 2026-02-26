// Chapter 6.5 — Non-Manifold & Edge Cases tests.
//
// Iteration deliverable:
//   - Zero-volume result detection
//   - Open-shell result handling
//   - Self-intersecting input detection with diagnostic
//
// Tests verify:
//   1. isZeroVolume returns true for an empty body
//   2. isZeroVolume returns false for a valid solid body
//   3. isZeroVolume returns true for BooleanDifference(A, A)
//   4. isZeroVolume returns true for BooleanIntersection of disjoint bodies
//   5. detectSelfIntersection returns false for a simple, clean box
//   6. detectSelfIntersection returns true for a body whose faces cross each other
//   7. detectSelfIntersection returns false for two parallel (non-crossing) planes
//   8. BooleanDifference of a body from itself is detected as zero-volume
//   9. BooleanUnion of disjoint bodies is detected as non-zero-volume

#include "GkTest.h"
#include "gk/boolean/BooleanOps.h"
#include "gk/brep/Body.h"
#include "gk/brep/BRepQuery.h"
#include "gk/brep/Face.h"
#include "gk/brep/Lump.h"
#include "gk/brep/Shell.h"
#include "gk/brep/Wire.h"
#include "gk/builders/PrimitiveBuilders.h"
#include "gk/surface/Plane.h"

using namespace gk;

// =============================================================================
// isZeroVolume
// =============================================================================

GK_TEST(Boolean6_5, ZeroVolume_EmptyBody_IsTrue)
{
    auto body = makeHandle<Body>();
    EXPECT_TRUE(isZeroVolume(body));
}

GK_TEST(Boolean6_5, ZeroVolume_ValidBox_IsFalse)
{
    auto box = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    EXPECT_FALSE(isZeroVolume(box));
}

GK_TEST(Boolean6_5, ZeroVolume_DifferenceWithSelf_IsTrue)
{
    auto box    = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    auto result = BooleanDifference(box, box);
    EXPECT_TRUE(isZeroVolume(result, 0.01, 20));
}

GK_TEST(Boolean6_5, ZeroVolume_DisjointIntersection_IsTrue)
{
    auto A = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    auto B = makeBox(Vec3{5, 0, 0}, Vec3{6, 1, 1});
    auto result = BooleanIntersection(A, B);
    EXPECT_TRUE(isZeroVolume(result));
}

// =============================================================================
// detectSelfIntersection
// =============================================================================

GK_TEST(Boolean6_5, SelfIntersect_CleanBox_IsFalse)
{
    // A clean box has no face-face curve intersections within the same shell
    auto box = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    EXPECT_FALSE(detectSelfIntersection(box));
}

GK_TEST(Boolean6_5, SelfIntersect_TwoCrossingPlanes_IsTrue)
{
    // Build a single-shell body with two planes that cross each other:
    //   Plane A: XY plane (z=0)
    //   Plane B: XZ plane (y=0)
    // These two planes intersect along the X axis → detectSelfIntersection == true
    auto shell = makeHandle<Shell>();
    shell->setClosed(false);

    auto makeInfiniteFace = [&](Vec3 origin, Vec3 u, Vec3 v) {
        auto f = makeHandle<Face>();
        f->setSurface(std::make_shared<Plane>(origin, u, v));
        f->setOrientation(FaceOrientation::kForward);
        f->setOuterWire(makeHandle<Wire>());
        return f;
    };

    shell->addFace(makeInfiniteFace(Vec3::zero(), Vec3::unitX(), Vec3::unitY()));  // XY
    shell->addFace(makeInfiniteFace(Vec3::zero(), Vec3::unitX(), Vec3::unitZ()));  // XZ

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);
    auto body = makeHandle<Body>();
    body->addLump(lump);

    EXPECT_TRUE(detectSelfIntersection(body));
}

GK_TEST(Boolean6_5, SelfIntersect_TwoParallelPlanes_IsFalse)
{
    // Two parallel planes do not intersect → detectSelfIntersection == false
    auto shell = makeHandle<Shell>();
    shell->setClosed(false);

    auto makeInfiniteFace = [&](Vec3 origin, Vec3 u, Vec3 v) {
        auto f = makeHandle<Face>();
        f->setSurface(std::make_shared<Plane>(origin, u, v));
        f->setOrientation(FaceOrientation::kForward);
        f->setOuterWire(makeHandle<Wire>());
        return f;
    };

    // Both faces are XY planes, offset along Z
    shell->addFace(makeInfiniteFace(Vec3::zero(),      Vec3::unitX(), Vec3::unitY()));
    shell->addFace(makeInfiniteFace(Vec3{0, 0, 1.0},   Vec3::unitX(), Vec3::unitY()));

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);
    auto body = makeHandle<Body>();
    body->addLump(lump);

    EXPECT_FALSE(detectSelfIntersection(body));
}

// =============================================================================
// Non-manifold result from Boolean
// =============================================================================

GK_TEST(Boolean6_5, BooleanDifference_SphereInsideBox_NonEmpty)
{
    auto box    = makeBox(Vec3{0, 0, 0}, Vec3{3, 3, 3});
    auto sphere = makeSphere(Vec3{1.5, 1.5, 1.5}, 0.5);
    auto result = BooleanDifference(box, sphere);
    EXPECT_FALSE(isZeroVolume(result));
}

GK_TEST(Boolean6_5, BooleanUnion_DisjointBodies_NonEmpty)
{
    auto A = makeBox(Vec3{0, 0, 0}, Vec3{1, 1, 1});
    auto B = makeBox(Vec3{2, 0, 0}, Vec3{3, 1, 1});
    auto result = BooleanUnion(A, B);
    EXPECT_FALSE(isZeroVolume(result));
}
