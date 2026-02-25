#include "GkTest.h"
#include "gk/surface/Plane.h"
#include "gk/surface/Sphere.h"
#include "gk/surface/Cylinder.h"
#include "gk/surface/Cone.h"
#include "gk/surface/Torus.h"
#include <cmath>
#include <utility>

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kTol = 1e-9;   // tolerance for all analytic checks

// ─── Helper: finite-difference check for partial derivatives ─────────────────
static void checkDerivatives(const gk::ISurface& surf, double u, double v,
                               double h = 1e-5, double tol = 1e-4)
{
    auto sp = surf.evaluate(u, v);
    auto p_uh = surf.evaluate(u+h, v).p;
    auto p_ul = surf.evaluate(u-h, v).p;
    auto p_vh = surf.evaluate(u, v+h).p;
    auto p_vl = surf.evaluate(u, v-h).p;

    gk::Vec3 du_fd = (p_uh - p_ul) * (1.0 / (2.0*h));
    gk::Vec3 dv_fd = (p_vh - p_vl) * (1.0 / (2.0*h));

    EXPECT_NEAR(sp.du.x, du_fd.x, tol);
    EXPECT_NEAR(sp.du.y, du_fd.y, tol);
    EXPECT_NEAR(sp.du.z, du_fd.z, tol);
    EXPECT_NEAR(sp.dv.x, dv_fd.x, tol);
    EXPECT_NEAR(sp.dv.y, dv_fd.y, tol);
    EXPECT_NEAR(sp.dv.z, dv_fd.z, tol);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Plane
// ═══════════════════════════════════════════════════════════════════════════════
GK_TEST(Plane, EvaluateXYPlane)
{
    auto p = gk::Plane::xyPlane();
    auto sp = p.evaluate(3.0, 4.0);
    GK_ASSERT_NEAR(sp.p.x, 3.0, kTol);
    GK_ASSERT_NEAR(sp.p.y, 4.0, kTol);
    GK_ASSERT_NEAR(sp.p.z, 0.0, kTol);
}

GK_TEST(Plane, DerivativesAreAxisVectors)
{
    auto p = gk::Plane::xyPlane();
    auto sp = p.evaluate(1.0, 2.0);
    GK_ASSERT_NEAR(sp.du.x, 1.0, kTol); GK_ASSERT_NEAR(sp.du.y, 0.0, kTol);
    GK_ASSERT_NEAR(sp.dv.x, 0.0, kTol); GK_ASSERT_NEAR(sp.dv.y, 1.0, kTol);
    // All second derivatives must be zero
    GK_ASSERT_NEAR(sp.duu.norm(), 0.0, kTol);
    GK_ASSERT_NEAR(sp.duv.norm(), 0.0, kTol);
    GK_ASSERT_NEAR(sp.dvv.norm(), 0.0, kTol);
}

GK_TEST(Plane, NormalIsZAxis)
{
    auto p = gk::Plane::xyPlane();
    auto n = p.normalAt(0.0, 0.0);
    GK_ASSERT_NEAR(n.x, 0.0, kTol);
    GK_ASSERT_NEAR(n.y, 0.0, kTol);
    GK_ASSERT_NEAR(n.z, 1.0, kTol);
}

GK_TEST(Plane, InverseEvaluate)
{
    auto p = gk::Plane::xzPlane();
    auto [u, v] = p.inverseEvaluate(gk::Vec3{5.0, 0.0, -3.0});
    GK_ASSERT_NEAR(u,  5.0, kTol);
    GK_ASSERT_NEAR(v, -3.0, kTol);
}

GK_TEST(Plane, ZeroCurvature)
{
    auto p = gk::Plane::yzPlane();
    GK_ASSERT_NEAR(p.gaussianCurvature(0,0), 0.0, kTol);
    GK_ASSERT_NEAR(p.meanCurvature(0,0),     0.0, kTol);
}

GK_TEST(Plane, IsNotClosed)
{
    bool cu, cv;
    gk::Plane::xyPlane().isClosed(cu, cv);
    GK_ASSERT_FALSE(cu);
    GK_ASSERT_FALSE(cv);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Sphere
// ═══════════════════════════════════════════════════════════════════════════════
GK_TEST(Sphere, EvaluateEquator)
{
    gk::Sphere s{gk::Vec3::zero(), 2.0};
    auto sp = s.evaluate(0.0, 0.0);
    // u=0,v=0 → (r,0,0)
    GK_ASSERT_NEAR(sp.p.x, 2.0, kTol);
    GK_ASSERT_NEAR(sp.p.y, 0.0, kTol);
    GK_ASSERT_NEAR(sp.p.z, 0.0, kTol);
}

GK_TEST(Sphere, EvaluateNorthPole)
{
    gk::Sphere s{gk::Vec3::zero(), 3.0};
    auto sp = s.evaluate(0.0, kPi/2.0);
    GK_ASSERT_NEAR(sp.p.x, 0.0, kTol);
    GK_ASSERT_NEAR(sp.p.y, 0.0, kTol);
    GK_ASSERT_NEAR(sp.p.z, 3.0, kTol);
}

GK_TEST(Sphere, NormalIsOutwardRadial)
{
    gk::Sphere s{gk::Vec3::zero(), 5.0};
    for (double u : {0.0, kPi/4, kPi/2, kPi})
    {
        for (double v : {-kPi/4, 0.0, kPi/4})
        {
            auto sp = s.evaluate(u, v);
            auto n  = s.normalAt(u, v);
            // n should point in same direction as (p - center)/r
            gk::Vec3 expected = sp.p * (1.0 / s.radius());
            GK_ASSERT_NEAR(n.x, expected.x, kTol);
            GK_ASSERT_NEAR(n.y, expected.y, kTol);
            GK_ASSERT_NEAR(n.z, expected.z, kTol);
        }
    }
}

GK_TEST(Sphere, Derivatives_FiniteDiff)
{
    gk::Sphere s{gk::Vec3{1,2,3}, 4.0};
    checkDerivatives(s, 0.5, 0.3);
    checkDerivatives(s, kPi, -0.1);
}

GK_TEST(Sphere, UVRoundTrip)
{
    gk::Sphere s{gk::Vec3::zero(), 7.0};
    for (double u : {0.1, 1.0, 2.5, 5.0})
    {
        for (double v : {-1.0, 0.0, 1.0})
        {
            auto sp = s.evaluate(u, v);
            auto [u2, v2] = s.inverseEvaluate(sp.p);
            GK_ASSERT_NEAR(u2, u, kTol);
            GK_ASSERT_NEAR(v2, v, kTol);
        }
    }
}

GK_TEST(Sphere, GaussianCurvature)
{
    double r = 3.0;
    gk::Sphere s{gk::Vec3::zero(), r};
    GK_ASSERT_NEAR(s.gaussianCurvature(0,0), 1.0/(r*r), kTol);
    GK_ASSERT_NEAR(s.gaussianCurvature(1,0), 1.0/(r*r), kTol);
}

GK_TEST(Sphere, MeanCurvature)
{
    double r = 4.0;
    gk::Sphere s{gk::Vec3::zero(), r};
    GK_ASSERT_NEAR(s.meanCurvature(0,0), 1.0/r, kTol);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Cylinder
// ═══════════════════════════════════════════════════════════════════════════════
GK_TEST(Cylinder, EvaluateAtAngleZero)
{
    gk::Cylinder c{gk::Vec3::zero(), gk::Vec3::unitZ(), 2.0, 0.0, 5.0};
    auto sp = c.evaluate(0.0, 1.5);
    GK_ASSERT_NEAR(sp.p.x, 2.0, kTol);
    GK_ASSERT_NEAR(sp.p.y, 0.0, kTol);
    GK_ASSERT_NEAR(sp.p.z, 1.5, kTol);
}

GK_TEST(Cylinder, NormalIsRadial)
{
    gk::Cylinder c{gk::Vec3::zero(), gk::Vec3::unitZ(), 3.0};
    for (double u : {0.0, kPi/4, kPi, 3*kPi/2})
    {
        auto n = c.normalAt(u, 0.5);
        // Normal is perpendicular to axis
        GK_ASSERT_NEAR(n.dot(gk::Vec3::unitZ()), 0.0, kTol);
        GK_ASSERT_NEAR(n.norm(), 1.0, kTol);
    }
}

GK_TEST(Cylinder, Derivatives_FiniteDiff)
{
    gk::Cylinder c{gk::Vec3{1,0,0}, gk::Vec3::unitZ(), 2.0, 0.0, 3.0};
    checkDerivatives(c, 0.5, 1.0);
}

GK_TEST(Cylinder, UVRoundTrip)
{
    gk::Cylinder c{gk::Vec3::zero(), gk::Vec3::unitZ(), 5.0};
    for (double u : {0.1, 1.5, 3.0, 5.5})
    {
        for (double v : {0.1, 0.5, 0.9})
        {
            auto sp = c.evaluate(u, v);
            auto [u2, v2] = c.inverseEvaluate(sp.p);
            GK_ASSERT_NEAR(u2, u, kTol);
            GK_ASSERT_NEAR(v2, v, kTol);
        }
    }
}

GK_TEST(Cylinder, ZeroGaussianCurvature)
{
    gk::Cylinder c{gk::Vec3::zero(), gk::Vec3::unitZ(), 2.0};
    GK_ASSERT_NEAR(c.gaussianCurvature(0,0), 0.0, kTol);
}

GK_TEST(Cylinder, MeanCurvature)
{
    double r = 3.0;
    gk::Cylinder c{gk::Vec3::zero(), gk::Vec3::unitZ(), r};
    GK_ASSERT_NEAR(c.meanCurvature(0,0), 0.5/r, kTol);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Cone
// ═══════════════════════════════════════════════════════════════════════════════
GK_TEST(Cone, EvaluateAtAngleZero)
{
    double alpha = kPi / 6.0;  // 30-degree half-angle
    gk::Cone c{gk::Vec3::zero(), gk::Vec3::unitZ(), alpha, 0.0, 2.0};
    auto sp = c.evaluate(0.0, 1.0);
    double r = std::tan(alpha);
    GK_ASSERT_NEAR(sp.p.x, r,   kTol);
    GK_ASSERT_NEAR(sp.p.y, 0.0, kTol);
    GK_ASSERT_NEAR(sp.p.z, 1.0, kTol);
}

GK_TEST(Cone, NormalIsUnitAndPerpToAxis)
{
    double alpha = kPi / 4.0;
    gk::Cone c{gk::Vec3::zero(), gk::Vec3::unitZ(), alpha};
    for (double u : {0.0, kPi/2, kPi})
    {
        auto n = c.normalAt(u, 1.0);
        GK_ASSERT_NEAR(n.norm(), 1.0, kTol);
        // Normal is perpendicular to the generatrix (axis + radial*tanA).
        // We verify via dot product with the known generatrix direction.
        double tanA = std::tan(alpha);
        gk::Vec3 gen = gk::Vec3::unitZ() + (gk::Vec3::unitX() * std::cos(u) +
                                              gk::Vec3::unitY() * std::sin(u)) * tanA;
        GK_ASSERT_NEAR(n.dot(gen.normalized()), 0.0, 1e-7);
    }
}

GK_TEST(Cone, Derivatives_FiniteDiff)
{
    double alpha = kPi / 5.0;
    gk::Cone c{gk::Vec3::zero(), gk::Vec3::unitZ(), alpha, 0.1, 2.0};
    checkDerivatives(c, 0.5, 1.0);
}

GK_TEST(Cone, UVRoundTrip)
{
    double alpha = kPi / 4.0;
    gk::Cone c{gk::Vec3::zero(), gk::Vec3::unitZ(), alpha, 0.0, 3.0};
    for (double u : {0.3, 1.2, 3.5})
    {
        for (double v : {0.5, 1.5, 2.5})
        {
            auto sp = c.evaluate(u, v);
            auto [u2, v2] = c.inverseEvaluate(sp.p);
            GK_ASSERT_NEAR(u2, u, kTol);
            GK_ASSERT_NEAR(v2, v, kTol);
        }
    }
}

GK_TEST(Cone, ZeroGaussianCurvature)
{
    gk::Cone c{gk::Vec3::zero(), gk::Vec3::unitZ(), kPi/4};
    GK_ASSERT_NEAR(c.gaussianCurvature(0,1), 0.0, kTol);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Torus
// ═══════════════════════════════════════════════════════════════════════════════
GK_TEST(Torus, EvaluateAtU0V0)
{
    gk::Torus t{gk::Vec3::zero(), gk::Vec3::unitZ(), 3.0, 1.0};
    auto sp = t.evaluate(0.0, 0.0);
    // u=0,v=0 → (R+r, 0, 0)
    GK_ASSERT_NEAR(sp.p.x, 4.0, kTol);
    GK_ASSERT_NEAR(sp.p.y, 0.0, kTol);
    GK_ASSERT_NEAR(sp.p.z, 0.0, kTol);
}

GK_TEST(Torus, EvaluateAtU90V0)
{
    gk::Torus t{gk::Vec3::zero(), gk::Vec3::unitZ(), 3.0, 1.0};
    auto sp = t.evaluate(kPi/2.0, 0.0);
    GK_ASSERT_NEAR(sp.p.x, 0.0, kTol);
    GK_ASSERT_NEAR(sp.p.y, 4.0, kTol);
    GK_ASSERT_NEAR(sp.p.z, 0.0, kTol);
}

GK_TEST(Torus, NormalAtU0V0)
{
    gk::Torus t{gk::Vec3::zero(), gk::Vec3::unitZ(), 3.0, 1.0};
    auto n = t.normalAt(0.0, 0.0);
    // At (R+r, 0, 0) the outward normal points in +X
    GK_ASSERT_NEAR(n.x, 1.0, kTol);
    GK_ASSERT_NEAR(n.y, 0.0, kTol);
    GK_ASSERT_NEAR(n.z, 0.0, kTol);
}

GK_TEST(Torus, Derivatives_FiniteDiff)
{
    gk::Torus t{gk::Vec3::zero(), gk::Vec3::unitZ(), 3.0, 1.0};
    checkDerivatives(t, 0.5, 0.7);
    checkDerivatives(t, kPi, kPi/3);
}

GK_TEST(Torus, UVRoundTrip)
{
    gk::Torus t{gk::Vec3::zero(), gk::Vec3::unitZ(), 3.0, 1.0};
    for (double u : {0.2, 1.5, 4.0})
    {
        for (double v : {0.3, 2.0, 5.5})
        {
            auto sp = t.evaluate(u, v);
            auto [u2, v2] = t.inverseEvaluate(sp.p);
            GK_ASSERT_NEAR(u2, u, kTol);
            GK_ASSERT_NEAR(v2, v, kTol);
        }
    }
}

GK_TEST(Torus, GaussianCurvatureAtV0)
{
    double R = 3.0, r = 1.0;
    gk::Torus t{gk::Vec3::zero(), gk::Vec3::unitZ(), R, r};
    // K(u,0) = cos(0) / (r*(R+r*cos(0))) = 1/(r*(R+r)) = 1/4
    double expected = 1.0 / (r * (R + r));
    GK_ASSERT_NEAR(t.gaussianCurvature(0.0, 0.0), expected, kTol);
}

GK_TEST(Torus, GaussianCurvatureAtVPi)
{
    double R = 3.0, r = 1.0;
    gk::Torus t{gk::Vec3::zero(), gk::Vec3::unitZ(), R, r};
    // K(u,π) = cos(π) / (r*(R+r*cos(π))) = -1/(r*(R-r)) = -1/2
    double expected = -1.0 / (r * (R - r));
    GK_ASSERT_NEAR(t.gaussianCurvature(0.0, kPi), expected, kTol);
}

GK_TEST(Torus, IsClosedInBothDirections)
{
    bool cu, cv;
    gk::Torus{gk::Vec3::zero(), gk::Vec3::unitZ(), 3.0, 1.0}.isClosed(cu, cv);
    GK_ASSERT_TRUE(cu);
    GK_ASSERT_TRUE(cv);
}
