#include "GkTest.h"
#include "gk/surface/Plane.h"
#include "gk/surface/Sphere.h"
#include "gk/surface/Cylinder.h"
#include "gk/surface/BSplineSurface.h"
#include "gk/surface/SurfaceUtils.h"
#include <cmath>
#include <vector>

static constexpr double kPi = 3.14159265358979323846;

using gk::BSplineSurface;

// ── Curvature via SurfaceUtils free functions ─────────────────────────────────

GK_TEST(SurfaceUtils, PlaneCurvatureIsZero)
{
    auto p = gk::Plane::xyPlane();
    EXPECT_NEAR(gk::gaussianCurvature(p, 0.0, 0.0), 0.0, 1e-12);
    EXPECT_NEAR(gk::meanCurvature   (p, 0.0, 0.0), 0.0, 1e-12);
}

GK_TEST(SurfaceUtils, SphereGaussianCurvatureViaFundForms)
{
    double r = 5.0;
    gk::Sphere s{gk::Vec3::zero(), r};
    // Free function uses fundamental forms, should match 1/r²
    double K = gk::gaussianCurvature(s, 0.3, 0.5);
    EXPECT_NEAR(K, 1.0/(r*r), 1e-7);
}

GK_TEST(SurfaceUtils, CylinderGaussianCurvatureIsZero)
{
    gk::Cylinder c{gk::Vec3::zero(), gk::Vec3::unitZ(), 3.0};
    EXPECT_NEAR(gk::gaussianCurvature(c, 0.5, 0.5), 0.0, 1e-9);
}

// ── Tessellate ────────────────────────────────────────────────────────────────

GK_TEST(SurfaceUtils, TessellateVertexCount)
{
    auto p = gk::Plane::xyPlane();
    auto m = gk::tessellate(p, 4, 3);
    // (4+1) * (3+1) = 20 vertices
    GK_ASSERT_EQ((int)m.vertices.size(), 20);
    // 4*3*2 = 24 triangles
    GK_ASSERT_EQ((int)m.triangles.size(), 24);
}

GK_TEST(SurfaceUtils, TessellateNormalsAreUnit)
{
    gk::Sphere s{gk::Vec3::zero(), 2.0};
    auto m = gk::tessellate(s, 8, 6);
    for (auto& n : m.normals)
    {
        EXPECT_NEAR(n.norm(), 1.0, 1e-9);
    }
}

GK_TEST(SurfaceUtils, TessellateVerticesLieOnSphere)
{
    double r = 3.0;
    gk::Sphere s{gk::Vec3::zero(), r};
    auto m = gk::tessellate(s, 6, 4);
    for (auto& v : m.vertices)
    {
        double dist = v.norm();
        EXPECT_NEAR(dist, r, 1e-9);
    }
}

GK_TEST(SurfaceUtils, TessellateInvalidStepsThrows)
{
    auto p = gk::Plane::xyPlane();
    bool threw = false;
    try { gk::tessellate(p, 0, 4); }
    catch (const std::invalid_argument&) { threw = true; }
    GK_ASSERT_TRUE(threw);
}

// ── closestPoint on Plane ─────────────────────────────────────────────────────

GK_TEST(SurfaceUtils, ClosestPointOnPlane)
{
    auto p = gk::Plane::xyPlane();
    // Point directly above (3, 4, 5) → closest (u,v) = (3, 4)
    auto [u, v] = gk::closestPoint(p, gk::Vec3{3.0, 4.0, 5.0}, 0.0, 0.0);
    EXPECT_NEAR(u, 3.0, 1e-7);
    EXPECT_NEAR(v, 4.0, 1e-7);
}

GK_TEST(SurfaceUtils, ClosestPointOnSphere)
{
    gk::Sphere s{gk::Vec3::zero(), 5.0};
    // External point is directly along +X; closest is at (u=0, v=0).
    auto [u, v] = gk::closestPoint(s, gk::Vec3{10.0, 0.0, 0.0}, 0.1, 0.0);
    auto cp = s.evaluate(u, v).p;
    EXPECT_NEAR(cp.x, 5.0, 1e-7);
    EXPECT_NEAR(cp.y, 0.0, 1e-7);
    EXPECT_NEAR(cp.z, 0.0, 1e-7);
}

GK_TEST(SurfaceUtils, ClosestPointOnBSpline)
{
    // Flat bilinear patch on [0,1]²; closest point to (0.3, 0.7, 5) is (0.3, 0.7).
    BSplineSurface::CtrlGrid pts(2, std::vector<gk::Vec3>(2));
    pts[0][0]={0,0,0}; pts[1][0]={1,0,0};
    pts[0][1]={0,1,0}; pts[1][1]={1,1,0};
    gk::BSplineSurface bs{1,1,{0,0,1,1},{0,0,1,1},pts};
    auto [u, v] = gk::closestPoint(bs, gk::Vec3{0.3, 0.7, 5.0}, 0.5, 0.5);
    EXPECT_NEAR(u, 0.3, 1e-7);
    EXPECT_NEAR(v, 0.7, 1e-7);
}

GK_TEST(SurfaceUtils, ClosestPointAlreadyOnSurface)
{
    auto p = gk::Plane::xyPlane();
    // Point on the plane; should converge to itself.
    auto [u, v] = gk::closestPoint(p, gk::Vec3{2.0, -1.0, 0.0}, 2.0, -1.0);
    EXPECT_NEAR(u,  2.0, 1e-9);
    EXPECT_NEAR(v, -1.0, 1e-9);
}
