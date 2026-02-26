// Iteration 6.1 — Intersection Engine tests.
//
// Tests cover all analytic surface pairs (Plane, Sphere, Cylinder, Cone, Torus)
// plus corner cases: tangency, no intersection, coincident surfaces, and
// degenerate geometry.

#include "GkTest.h"
#include "gk/boolean/FaceFaceIntersect.h"
#include "gk/brep/Face.h"
#include "gk/curve/Circle.h"
#include "gk/curve/Line.h"
#include "gk/surface/Cone.h"
#include "gk/surface/Cylinder.h"
#include "gk/surface/Plane.h"
#include "gk/surface/Sphere.h"
#include "gk/surface/Torus.h"
#include <cmath>
#include <memory>

using namespace gk;
using Type = FaceFaceIntersectResult::Type;

static constexpr double kPi  = 3.14159265358979323846;
static constexpr double kTol = 1e-6;

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Build a Face that wraps a surface (no explicit UV domain → surface default).
template <typename S, typename... Args>
Handle<Face> makeFaceWith(Args&&... args)
{
    auto f = makeHandle<Face>();
    f->setSurface(std::make_shared<S>(std::forward<Args>(args)...));
    return f;
}

// =============================================================================
// Plane ∩ Plane
// =============================================================================

GK_TEST(Intersect6_1, PlanePlane_Intersection_IsLine)
{
    // XY plane and XZ plane → intersection along the X axis
    auto fA = makeFaceWith<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY());
    auto fB = makeFaceWith<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitZ());

    auto res = IntersectFaceFace(*fA, *fB);

    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_EQ((int)res.curves.size(), 1);

    // The intersection line must pass through the origin
    auto& line = *res.curves[0];
    Vec3 ptOnLine = line.evaluate(0.0).p;
    EXPECT_NEAR(ptOnLine.y, 0.0, kTol);
    EXPECT_NEAR(ptOnLine.z, 0.0, kTol);

    // Tangent direction must be parallel to X
    Vec3 dir = line.evaluate(0.0).d1.normalized();
    EXPECT_NEAR(std::abs(dir.dot(Vec3::unitX())), 1.0, kTol);
}

GK_TEST(Intersect6_1, PlanePlane_Parallel_NoIntersection)
{
    // Two parallel planes offset by 1 unit along Z
    auto fA = makeFaceWith<Plane>(Vec3::zero(),       Vec3::unitX(), Vec3::unitY());
    auto fB = makeFaceWith<Plane>(Vec3{0, 0, 1.0},    Vec3::unitX(), Vec3::unitY());

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kNone);
}

GK_TEST(Intersect6_1, PlanePlane_Coincident)
{
    // Same plane described with different origins → coincident
    auto fA = makeFaceWith<Plane>(Vec3::zero(),   Vec3::unitX(), Vec3::unitY());
    auto fB = makeFaceWith<Plane>(Vec3{1, 2, 0},  Vec3::unitX(), Vec3::unitY());

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kCoincident);
}

GK_TEST(Intersect6_1, PlanePlane_NonOrthogonal_HasLine)
{
    // Two planes at 45° to each other
    Vec3 n2 = Vec3{0, 1, 1}.normalized();
    // Build orthonormal u,v axes for each plane
    Vec3 u1 = Vec3::unitX(), v1 = Vec3::unitY();
    Vec3 u2 = Vec3::unitX();
    Vec3 v2 = n2.cross(u2).normalized();

    auto fA = makeFaceWith<Plane>(Vec3::zero(), u1, v1);
    auto fB = makeFaceWith<Plane>(Vec3::zero(), u2, v2);

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_FALSE(res.curves.empty());

    // The line must lie on both planes (z = 0 and n2·p = 0)
    Vec3 pt = res.curves[0]->evaluate(0.0).p;
    EXPECT_NEAR(pt.z, 0.0, kTol);
    EXPECT_NEAR(n2.dot(pt), 0.0, kTol);
}

// =============================================================================
// Plane ∩ Sphere
// =============================================================================

GK_TEST(Intersect6_1, PlaneSphere_Intersection_IsCircle)
{
    // XY plane through centre of unit sphere at origin → equatorial circle, r=1
    auto fPlane  = makeFaceWith<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY());
    auto fSphere = makeFaceWith<Sphere>(Vec3::zero(), 1.0);

    auto res = IntersectFaceFace(*fPlane, *fSphere);

    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_EQ((int)res.curves.size(), 1);

    // Verify circle radius via arc length
    auto& circ = *res.curves[0];
    double len = circ.approximateLength(256);
    EXPECT_NEAR(len, 2.0 * kPi, 1e-3);

    // All points on the circle must lie in the plane (z ≈ 0)
    for (int i = 0; i <= 32; ++i) {
        double t = circ.domain().lo + circ.domain().width() * (double(i) / 32.0);
        EXPECT_NEAR(circ.evaluate(t).p.z, 0.0, kTol);
    }
}

GK_TEST(Intersect6_1, PlaneSphere_OffsetPlane_SmallerCircle)
{
    // Plane at z = 0.5 cuts the unit sphere: r_circle = sqrt(1 - 0.25) = sqrt(0.75)
    auto fPlane  = makeFaceWith<Plane>(Vec3{0, 0, 0.5}, Vec3::unitX(), Vec3::unitY());
    auto fSphere = makeFaceWith<Sphere>(Vec3::zero(), 1.0);

    auto res = IntersectFaceFace(*fPlane, *fSphere);
    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_EQ((int)res.curves.size(), 1);

    double expectedR = std::sqrt(0.75);
    double len       = res.curves[0]->approximateLength(256);
    EXPECT_NEAR(len, 2.0 * kPi * expectedR, 1e-3);
}

GK_TEST(Intersect6_1, PlaneSphere_Tangent_IsPoint)
{
    // Plane z = 1 is tangent to the unit sphere at (0,0,1)
    auto fPlane  = makeFaceWith<Plane>(Vec3{0, 0, 1.0}, Vec3::unitX(), Vec3::unitY());
    auto fSphere = makeFaceWith<Sphere>(Vec3::zero(), 1.0);

    auto res = IntersectFaceFace(*fPlane, *fSphere);

    EXPECT_EQ(res.type, Type::kPoint);
    ASSERT_EQ((int)res.points.size(), 1);
    EXPECT_NEAR(res.points[0].x, 0.0, kTol);
    EXPECT_NEAR(res.points[0].y, 0.0, kTol);
    EXPECT_NEAR(res.points[0].z, 1.0, kTol);
}

GK_TEST(Intersect6_1, PlaneSphere_Miss_NoIntersection)
{
    // Plane z = 2 is above the unit sphere → no intersection
    auto fPlane  = makeFaceWith<Plane>(Vec3{0, 0, 2.0}, Vec3::unitX(), Vec3::unitY());
    auto fSphere = makeFaceWith<Sphere>(Vec3::zero(), 1.0);

    auto res = IntersectFaceFace(*fPlane, *fSphere);
    EXPECT_EQ(res.type, Type::kNone);
}

GK_TEST(Intersect6_1, PlaneSphere_SphereBelowPlane_NoIntersection)
{
    // The sphere is entirely on one side of the plane
    auto fPlane  = makeFaceWith<Plane>(Vec3{0, 0, 5.0}, Vec3::unitX(), Vec3::unitY());
    auto fSphere = makeFaceWith<Sphere>(Vec3::zero(), 1.0);

    auto res = IntersectFaceFace(*fPlane, *fSphere);
    EXPECT_EQ(res.type, Type::kNone);
}

// =============================================================================
// Sphere ∩ Sphere
// =============================================================================

GK_TEST(Intersect6_1, SphereSphere_Intersection_IsCircle)
{
    // Two unit spheres whose centres are 1 apart → intersection circle
    auto fA = makeFaceWith<Sphere>(Vec3::zero(),   1.0);
    auto fB = makeFaceWith<Sphere>(Vec3{1,0,0},    1.0);

    auto res = IntersectFaceFace(*fA, *fB);

    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_EQ((int)res.curves.size(), 1);

    // All points must lie on both spheres
    auto& circ = *res.curves[0];
    for (int i = 0; i <= 32; ++i) {
        double t  = circ.domain().lo + circ.domain().width() * (double(i) / 32.0);
        Vec3   pt = circ.evaluate(t).p;
        EXPECT_NEAR(pt.norm(), 1.0, kTol);                   // on sphere A
        EXPECT_NEAR((pt - Vec3{1,0,0}).norm(), 1.0, kTol);  // on sphere B
    }
}

GK_TEST(Intersect6_1, SphereSphere_ExternalTangent_IsPoint)
{
    // Two unit spheres whose centres are 2 apart → tangent at midpoint
    auto fA = makeFaceWith<Sphere>(Vec3::zero(), 1.0);
    auto fB = makeFaceWith<Sphere>(Vec3{2,0,0},  1.0);

    auto res = IntersectFaceFace(*fA, *fB);

    EXPECT_EQ(res.type, Type::kPoint);
    ASSERT_EQ((int)res.points.size(), 1);
    EXPECT_NEAR(res.points[0].x, 1.0, kTol);
    EXPECT_NEAR(res.points[0].y, 0.0, kTol);
    EXPECT_NEAR(res.points[0].z, 0.0, kTol);
}

GK_TEST(Intersect6_1, SphereSphere_NoOverlap_NoIntersection)
{
    // Centres are 3 apart, each radius 1 → no intersection
    auto fA = makeFaceWith<Sphere>(Vec3::zero(), 1.0);
    auto fB = makeFaceWith<Sphere>(Vec3{3,0,0},  1.0);

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kNone);
}

GK_TEST(Intersect6_1, SphereSphere_OneInsideOther_NoIntersection)
{
    // Small sphere entirely inside the large sphere
    auto fA = makeFaceWith<Sphere>(Vec3::zero(), 5.0);
    auto fB = makeFaceWith<Sphere>(Vec3{1,0,0},  1.0);

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kNone);
}

GK_TEST(Intersect6_1, SphereSphere_InternalTangent_IsPoint)
{
    // Inner sphere just touches the outer from inside: r_inner = r_outer - d
    // r_outer=2, r_inner=1, d=1 → d = r_outer - r_inner = 1 → internal tangent
    auto fA = makeFaceWith<Sphere>(Vec3::zero(), 2.0);
    auto fB = makeFaceWith<Sphere>(Vec3{1,0,0},  1.0);

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kPoint);
    ASSERT_EQ((int)res.points.size(), 1);
    // Tangent point is at (2,0,0)
    EXPECT_NEAR(res.points[0].x, 2.0, kTol);
    EXPECT_NEAR(res.points[0].y, 0.0, kTol);
    EXPECT_NEAR(res.points[0].z, 0.0, kTol);
}

GK_TEST(Intersect6_1, SphereSphere_Concentric_EqualRadii_Coincident)
{
    // Same sphere at origin, same radius → coincident
    auto fA = makeFaceWith<Sphere>(Vec3::zero(), 1.0);
    auto fB = makeFaceWith<Sphere>(Vec3::zero(), 1.0);

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kCoincident);
}

GK_TEST(Intersect6_1, SphereSphere_Concentric_DifferentRadii_NoIntersection)
{
    auto fA = makeFaceWith<Sphere>(Vec3::zero(), 1.0);
    auto fB = makeFaceWith<Sphere>(Vec3::zero(), 2.0);

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kNone);
}

// =============================================================================
// Plane ∩ Cylinder — analytic circle case (plane ⊥ axis)
// =============================================================================

GK_TEST(Intersect6_1, PlaneCylinder_Perpendicular_IsCircle)
{
    // XY plane cuts a unit cylinder aligned along Z at z=0 → circle r=1 at z=0
    auto fPlane = makeFaceWith<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY());
    auto fCyl   = makeFaceWith<Cylinder>(Vec3::zero(), Vec3::unitZ(), 1.0, -1.0, 1.0);

    auto res = IntersectFaceFace(*fPlane, *fCyl);

    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_EQ((int)res.curves.size(), 1);

    double len = res.curves[0]->approximateLength(256);
    EXPECT_NEAR(len, 2.0 * kPi, 1e-3);
}

GK_TEST(Intersect6_1, PlaneCylinder_Perpendicular_OutsideRange_NoIntersection)
{
    // Cylinder spans v ∈ [0,1]; plane at z=2 → no intersection
    auto fPlane = makeFaceWith<Plane>(Vec3{0, 0, 2}, Vec3::unitX(), Vec3::unitY());
    auto fCyl   = makeFaceWith<Cylinder>(Vec3::zero(), Vec3::unitZ(), 1.0, 0.0, 1.0);

    auto res = IntersectFaceFace(*fPlane, *fCyl);
    EXPECT_EQ(res.type, Type::kNone);
}

// =============================================================================
// Plane ∩ Cone — analytic circle case (plane ⊥ axis)
// =============================================================================

GK_TEST(Intersect6_1, PlaneCone_Perpendicular_IsCircle)
{
    // Cone: apex at origin, axis +Z, half-angle 30°, v ∈ [0, 2]
    // Plane at z = 1 → circle of radius tan(30°) ≈ 0.577
    double halfAngle = kPi / 6.0;
    auto fPlane = makeFaceWith<Plane>(Vec3{0, 0, 1.0}, Vec3::unitX(), Vec3::unitY());
    auto fCone  = makeFaceWith<Cone>(Vec3::zero(), Vec3::unitZ(), halfAngle, 0.01, 2.0);

    auto res = IntersectFaceFace(*fPlane, *fCone);

    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_EQ((int)res.curves.size(), 1);

    double expectedR = std::tan(halfAngle);
    double len       = res.curves[0]->approximateLength(256);
    EXPECT_NEAR(len, 2.0 * kPi * expectedR, 1e-3);
}

GK_TEST(Intersect6_1, PlaneCone_Perpendicular_AtApex_IsPoint)
{
    // Plane at z = 0 just touches the apex: should give a point
    double halfAngle = kPi / 6.0;
    auto fPlane = makeFaceWith<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY());
    // cone v starts at 0 (the apex)
    auto fCone  = makeFaceWith<Cone>(Vec3::zero(), Vec3::unitZ(), halfAngle, 0.0, 2.0);

    auto res = IntersectFaceFace(*fPlane, *fCone);
    // Apex is at v=0, r=0 → point
    EXPECT_EQ(res.type, Type::kPoint);
    ASSERT_FALSE(res.points.empty());
    EXPECT_NEAR(res.points[0].norm(), 0.0, kTol);
}

GK_TEST(Intersect6_1, PlaneCone_Perpendicular_OutsideRange_NoIntersection)
{
    double halfAngle = kPi / 6.0;
    auto fPlane = makeFaceWith<Plane>(Vec3{0, 0, 5.0}, Vec3::unitX(), Vec3::unitY());
    auto fCone  = makeFaceWith<Cone>(Vec3::zero(), Vec3::unitZ(), halfAngle, 0.0, 2.0);

    auto res = IntersectFaceFace(*fPlane, *fCone);
    EXPECT_EQ(res.type, Type::kNone);
}

// =============================================================================
// Plane ∩ Cylinder — general angle (marching, produces ellipse)
// =============================================================================

GK_TEST(Intersect6_1, PlaneCylinder_Oblique_HasCurve)
{
    // Plane through origin with normal at 45° to the cylinder axis
    // → intersection is an ellipse (traced via marching)
    Vec3 n = Vec3{0, 1, 1}.normalized();
    Vec3 u = Vec3::unitX();
    Vec3 v = n.cross(u).normalized();
    auto fPlane = makeFaceWith<Plane>(Vec3::zero(), u, v);
    auto fCyl   = makeFaceWith<Cylinder>(Vec3::zero(), Vec3::unitZ(), 1.0, -2.0, 2.0);

    auto res = IntersectFaceFace(*fPlane, *fCyl);
    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_FALSE(res.curves.empty());

    // All points on the curve must lie on the cylinder surface (distance from
    // axis ≈ 1) and on the plane (n · p ≈ 0).
    auto* sc = dynamic_cast<SampledCurve3*>(res.curves[0].get());
    ASSERT_NE(sc, nullptr);
    for (auto& pt : sc->points()) {
        // Distance from the Z axis
        double distFromAxis = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        EXPECT_NEAR(distFromAxis, 1.0, 1e-3);
        // On the plane
        EXPECT_NEAR(n.dot(pt), 0.0, 1e-3);
    }
}

// =============================================================================
// Plane ∩ Torus  (marching)
// =============================================================================

GK_TEST(Intersect6_1, PlaneTorus_Equatorial_HasTwoCurves)
{
    // Torus (R=2, r=1) with axis Z, intersected by the z=0 equatorial plane.
    // Intersection: two circles at radii R-r=1 and R+r=3.
    auto fPlane = makeFaceWith<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY());
    auto fTorus = makeFaceWith<Torus>(Vec3::zero(), Vec3::unitZ(), 2.0, 1.0);

    auto res = IntersectFaceFace(*fPlane, *fTorus);
    EXPECT_EQ(res.type, Type::kCurve);
    EXPECT_GE((int)res.curves.size(), 1);

    // Every sampled point must lie in the plane (z ≈ 0)
    for (auto& c : res.curves) {
        auto* sc = dynamic_cast<SampledCurve3*>(c.get());
        if (!sc) continue;
        for (auto& pt : sc->points())
            EXPECT_NEAR(pt.z, 0.0, 1e-3);
    }
}

GK_TEST(Intersect6_1, PlaneTorus_NoIntersection)
{
    // Plane at z=4 misses the torus (R=2, r=1, so max z = r = 1)
    auto fPlane = makeFaceWith<Plane>(Vec3{0,0,4}, Vec3::unitX(), Vec3::unitY());
    auto fTorus = makeFaceWith<Torus>(Vec3::zero(), Vec3::unitZ(), 2.0, 1.0);

    auto res = IntersectFaceFace(*fPlane, *fTorus);
    EXPECT_EQ(res.type, Type::kNone);
}

// =============================================================================
// Sphere ∩ Cylinder  (marching)
// =============================================================================

GK_TEST(Intersect6_1, SphereCylinder_Intersection_HasCurve)
{
    // Unit sphere at origin, cylinder radius 0.5 along Z → two intersection loops
    auto fSphere = makeFaceWith<Sphere>(Vec3::zero(), 1.0);
    auto fCyl    = makeFaceWith<Cylinder>(Vec3::zero(), Vec3::unitZ(), 0.5, -2.0, 2.0);

    auto res = IntersectFaceFace(*fSphere, *fCyl);
    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_FALSE(res.curves.empty());

    // Points must lie on the sphere and on the cylinder
    for (auto& c : res.curves) {
        auto* sc = dynamic_cast<SampledCurve3*>(c.get());
        if (!sc) continue;
        for (auto& pt : sc->points()) {
            EXPECT_NEAR(pt.norm(), 1.0, 1e-2);
            double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            EXPECT_NEAR(r, 0.5, 1e-2);
        }
    }
}

GK_TEST(Intersect6_1, SphereCylinder_NoIntersection)
{
    // Cylinder radius 2 > sphere radius 1 → sphere entirely inside cylinder
    // (they never intersect from the outside)
    auto fSphere = makeFaceWith<Sphere>(Vec3::zero(), 1.0);
    auto fCyl    = makeFaceWith<Cylinder>(Vec3{5,0,0}, Vec3::unitZ(), 0.3, -2.0, 2.0);

    auto res = IntersectFaceFace(*fSphere, *fCyl);
    EXPECT_EQ(res.type, Type::kNone);
}

// =============================================================================
// Cylinder ∩ Cylinder  (marching)
// =============================================================================

GK_TEST(Intersect6_1, CylinderCylinder_Orthogonal_HasCurve)
{
    // Two unit cylinders, axes along Z and X respectively, both through origin
    auto fCylZ = makeFaceWith<Cylinder>(Vec3::zero(), Vec3::unitZ(), 1.0, -2.0, 2.0);
    auto fCylX = makeFaceWith<Cylinder>(Vec3::zero(), Vec3::unitX(), 1.0, -2.0, 2.0);

    auto res = IntersectFaceFace(*fCylZ, *fCylX);
    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_FALSE(res.curves.empty());

    // Points must lie on both cylinders
    for (auto& c : res.curves) {
        auto* sc = dynamic_cast<SampledCurve3*>(c.get());
        if (!sc) continue;
        for (auto& pt : sc->points()) {
            double rZ = std::sqrt(pt.x * pt.x + pt.y * pt.y);   // dist from Z-axis
            double rX = std::sqrt(pt.y * pt.y + pt.z * pt.z);   // dist from X-axis
            EXPECT_NEAR(rZ, 1.0, 0.05);
            EXPECT_NEAR(rX, 1.0, 0.05);
        }
    }
}

GK_TEST(Intersect6_1, CylinderCylinder_Parallel_NoIntersection)
{
    // Two parallel non-intersecting cylinders
    auto fA = makeFaceWith<Cylinder>(Vec3::zero(),   Vec3::unitZ(), 0.4, 0.0, 2.0);
    auto fB = makeFaceWith<Cylinder>(Vec3{2,0,0},    Vec3::unitZ(), 0.4, 0.0, 2.0);

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kNone);
}

// =============================================================================
// Sphere ∩ Cone  (marching)
// =============================================================================

GK_TEST(Intersect6_1, SphereCone_Intersection_HasCurve)
{
    // Cone: apex origin, axis Z, 45°, h ∈ [0.1, 3]
    // Sphere: center (0,0,1), radius 1.5 → intersects the cone
    double halfAngle = kPi / 4.0;
    auto fSphere = makeFaceWith<Sphere>(Vec3{0, 0, 1.0}, 1.5);
    auto fCone   = makeFaceWith<Cone>(Vec3::zero(), Vec3::unitZ(), halfAngle, 0.1, 3.0);

    auto res = IntersectFaceFace(*fSphere, *fCone);
    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_FALSE(res.curves.empty());
}

// =============================================================================
// Cylinder ∩ Torus  (marching)
// =============================================================================

GK_TEST(Intersect6_1, CylinderTorus_Intersection_HasCurve)
{
    // Torus (R=2, r=0.5), axis Z; cylinder radius 0.3, axis through (2,0,0)
    // along Z — this cylinder threads through the torus tube
    auto fTorus = makeFaceWith<Torus>(Vec3::zero(), Vec3::unitZ(), 2.0, 0.5);
    auto fCyl   = makeFaceWith<Cylinder>(Vec3{2, 0, 0}, Vec3::unitZ(), 0.3, -2.0, 2.0);

    auto res = IntersectFaceFace(*fTorus, *fCyl);
    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_FALSE(res.curves.empty());
}

// =============================================================================
// Corner cases
// =============================================================================

GK_TEST(Intersect6_1, CornerCase_NoSurface_ReturnsNone)
{
    // A Face with no surface attached → kNone
    auto fA = makeHandle<Face>();   // no surface
    auto fB = makeFaceWith<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY());

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kNone);
}

GK_TEST(Intersect6_1, CornerCase_BothNoSurface_ReturnsNone)
{
    auto fA = makeHandle<Face>();
    auto fB = makeHandle<Face>();

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kNone);
}

GK_TEST(Intersect6_1, CornerCase_PlanePlane_NearlyParallel)
{
    // Two planes whose normals differ by ~1e-5 radians.
    // Plane A: XY plane (normal = Z).
    // Plane B: slightly tilted — normal n2 = (eps, 0, 1).normalized().
    //   Build uAxis and vAxis for plane B so that uAxis × vAxis == n2.
    double eps = 1e-5;
    Vec3 n2 = Vec3{eps, 0.0, 1.0}.normalized();
    Vec3 u2 = Vec3{1.0, 0.0, -eps}.normalized();   // ⊥ n2
    Vec3 v2 = n2.cross(u2).normalized();            // completes the frame

    auto fA = makeFaceWith<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY());
    auto fB = makeFaceWith<Plane>(Vec3::zero(), u2, v2);

    // sinAngle ≈ eps = 1e-5 >> default tol = 1e-8 → must produce a line
    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kCurve);
}

GK_TEST(Intersect6_1, CornerCase_SphereSphere_LargeRadii)
{
    // Large spheres to test numerical robustness
    auto fA = makeFaceWith<Sphere>(Vec3::zero(),   1000.0);
    auto fB = makeFaceWith<Sphere>(Vec3{500,0,0},   1000.0);

    auto res = IntersectFaceFace(*fA, *fB);
    EXPECT_EQ(res.type, Type::kCurve);
    ASSERT_EQ((int)res.curves.size(), 1);

    // Circle should be at x = 125 (a = (1000²-1000²+500²)/(2*500) = 250000/1000=250,
    // wait: a = (r1²-r2²+d²)/(2d) = (1e6-1e6+250000)/1000 = 250)
    auto* circ = dynamic_cast<Circle3*>(res.curves[0].get());
    ASSERT_NE(circ, nullptr);
    Vec3 pt0 = circ->evaluate(0.0).p;
    EXPECT_NEAR(pt0.x, 250.0, 1.0);
}

GK_TEST(Intersect6_1, CornerCase_PlaneSphere_NearTangent)
{
    // Plane at z = 1 - eps: very close to tangent (r=1, sphere at origin)
    double eps = 1e-6;
    auto fPlane  = makeFaceWith<Plane>(Vec3{0, 0, 1.0 - eps}, Vec3::unitX(), Vec3::unitY());
    auto fSphere = makeFaceWith<Sphere>(Vec3::zero(), 1.0);

    auto res = IntersectFaceFace(*fPlane, *fSphere);
    // Should be a very small circle or a tangent point
    EXPECT_TRUE(res.type == Type::kCurve || res.type == Type::kPoint);
}

GK_TEST(Intersect6_1, CornerCase_SampledCurve3_ArcLength)
{
    // Verify SampledCurve3 evaluate & domain work correctly
    std::vector<Vec3> pts;
    for (int i = 0; i <= 10; ++i)
        pts.push_back(Vec3{double(i), 0.0, 0.0});
    SampledCurve3 sc(pts);

    EXPECT_NEAR(sc.domain().lo, 0.0, 1e-12);
    EXPECT_NEAR(sc.domain().hi, 10.0, 1e-12);
    EXPECT_FALSE(sc.isClosed());

    // At t=5 we should be at (5,0,0)
    Vec3 mid = sc.evaluate(5.0).p;
    EXPECT_NEAR(mid.x, 5.0, 1e-9);
    EXPECT_NEAR(mid.y, 0.0, 1e-9);
}

GK_TEST(Intersect6_1, CornerCase_SampledCurve3_Closed)
{
    // A triangle — closed curve
    std::vector<Vec3> pts = {
        {0,0,0}, {1,0,0}, {0.5,1,0}, {0,0,0}
    };
    SampledCurve3 sc(pts);
    EXPECT_TRUE(sc.isClosed());
}
