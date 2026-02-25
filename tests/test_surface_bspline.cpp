#include "GkTest.h"
#include "StepWriter.h"
#include "gk/surface/BSplineSurface.h"
#include "gk/surface/SurfaceUtils.h"
#include <cmath>
#include <vector>

using gk::BSplineSurface;
using gk::Vec3;

// ── Helper: compare two Vec3 values ──────────────────────────────────────────
static void expectVec3Near(const Vec3& a, const Vec3& b, double tol = 1e-9)
{
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
}

// ── Helper: finite-difference derivative check for BSplineSurface ────────────
static void checkBSplineDerivs(const BSplineSurface& s, double u, double v,
                                 double h = 1e-5, double tol = 1e-4)
{
    auto sp = s.evaluate(u, v);
    auto du_fd = (s.evaluate(u+h,v).p - s.evaluate(u-h,v).p) * (1.0/(2.0*h));
    auto dv_fd = (s.evaluate(u,v+h).p - s.evaluate(u,v-h).p) * (1.0/(2.0*h));
    EXPECT_NEAR(sp.du.x, du_fd.x, tol);
    EXPECT_NEAR(sp.du.y, du_fd.y, tol);
    EXPECT_NEAR(sp.du.z, du_fd.z, tol);
    EXPECT_NEAR(sp.dv.x, dv_fd.x, tol);
    EXPECT_NEAR(sp.dv.y, dv_fd.y, tol);
    EXPECT_NEAR(sp.dv.z, dv_fd.z, tol);
}

// ── Bilinear patch (degree 1 × 1) ─────────────────────────────────────────────
BSplineSurface makeBilinear()
{
    // 4 control points at unit square corners, degree 1 × 1
    // Knots: [0,0,1,1] in both directions
    BSplineSurface::CtrlGrid pts(2, std::vector<Vec3>(2));
    pts[0][0] = Vec3{0,0,0};
    pts[1][0] = Vec3{1,0,0};
    pts[0][1] = Vec3{0,1,0};
    pts[1][1] = Vec3{1,1,0};
    return BSplineSurface{1, 1,
                          {0.0,0.0,1.0,1.0},
                          {0.0,0.0,1.0,1.0},
                          pts};
}

GK_TEST(BSplineSurface, BilinearCornersMatchControlPoints)
{
    auto s = makeBilinear();
    expectVec3Near(s.evaluate(0.0, 0.0).p, Vec3{0,0,0});
    expectVec3Near(s.evaluate(1.0, 0.0).p, Vec3{1,0,0});
    expectVec3Near(s.evaluate(0.0, 1.0).p, Vec3{0,1,0});
    expectVec3Near(s.evaluate(1.0, 1.0).p, Vec3{1,1,0});
}

GK_TEST(BSplineSurface, BilinearMidpointIsAverage)
{
    auto s = makeBilinear();
    expectVec3Near(s.evaluate(0.5, 0.5).p, Vec3{0.5, 0.5, 0.0});
}

GK_TEST(BSplineSurface, BilinearDerivativesConstant)
{
    auto s = makeBilinear();
    auto sp = s.evaluate(0.5, 0.5);
    // For a flat bilinear patch on the unit square:  du=(1,0,0), dv=(0,1,0)
    expectVec3Near(sp.du, Vec3{1,0,0});
    expectVec3Near(sp.dv, Vec3{0,1,0});
}

GK_TEST(BSplineSurface, BilinearDomain)
{
    auto s = makeBilinear();
    auto dom = s.domain();
    EXPECT_NEAR(dom.u.lo, 0.0, 1e-12); EXPECT_NEAR(dom.u.hi, 1.0, 1e-12);
    EXPECT_NEAR(dom.v.lo, 0.0, 1e-12); EXPECT_NEAR(dom.v.hi, 1.0, 1e-12);
}

// ── Non-planar quadratic × linear patch ───────────────────────────────────────
BSplineSurface makeQuadLinear()
{
    // 3 × 2 control points, degree 2 in u, degree 1 in v.
    // Control net: a paraboloid-like shape.
    BSplineSurface::CtrlGrid pts(3, std::vector<Vec3>(2));
    pts[0][0] = Vec3{0, 0, 0};
    pts[1][0] = Vec3{1, 0, 1};   // lifted mid-row
    pts[2][0] = Vec3{2, 0, 0};
    pts[0][1] = Vec3{0, 1, 0};
    pts[1][1] = Vec3{1, 1, 1};
    pts[2][1] = Vec3{2, 1, 0};
    return BSplineSurface{2, 1,
                          {0.0,0.0,0.0,1.0,1.0,1.0},
                          {0.0,0.0,1.0,1.0},
                          pts};
}

GK_TEST(BSplineSurface, QuadLinearDerivatives_FiniteDiff)
{
    auto s = makeQuadLinear();
    checkBSplineDerivs(s, 0.3, 0.5);
    checkBSplineDerivs(s, 0.7, 0.2);
    checkBSplineDerivs(s, 0.5, 0.5);
}

GK_TEST(BSplineSurface, QuadLinearApexElevation)
{
    auto s = makeQuadLinear();
    // At u=0.5,v=0 the quadratic arc peaks at z=0.5 (Bernstein basis midpoint)
    auto sp = s.evaluate(0.5, 0.0);
    EXPECT_NEAR(sp.p.z, 0.5, 1e-9);
}

// ── Uniform knot helper ───────────────────────────────────────────────────────
GK_TEST(BSplineSurface, UniformKnotsSize)
{
    auto kv = BSplineSurface::uniformKnots(5, 3);
    // size should be 5+3+1 = 9
    GK_ASSERT_EQ((int)kv.size(), 9);
    EXPECT_NEAR(kv.front(), 0.0, 1e-12);
    EXPECT_NEAR(kv.back(),  1.0, 1e-12);
}

// ── Knot insertion in U ───────────────────────────────────────────────────────
GK_TEST(BSplineSurface, KnotInsertionU_PreservesGeometry)
{
    auto s = makeQuadLinear();
    auto s2 = s.insertKnotU(0.5);

    // After insertion the new surface must evaluate to the same points.
    for (double u : {0.1, 0.3, 0.5, 0.7, 0.9})
        for (double v : {0.0, 0.5, 1.0})
            expectVec3Near(s.evaluate(u,v).p, s2.evaluate(u,v).p, 1e-9);

    // One more control point and knot entry
    GK_ASSERT_EQ(s2.numU(), s.numU() + 1);
}

// ── Knot insertion in V ───────────────────────────────────────────────────────
GK_TEST(BSplineSurface, KnotInsertionV_PreservesGeometry)
{
    auto s = makeQuadLinear();
    auto s2 = s.insertKnotV(0.4);

    for (double u : {0.1, 0.5, 0.9})
        for (double v : {0.1, 0.4, 0.8})
            expectVec3Near(s.evaluate(u,v).p, s2.evaluate(u,v).p, 1e-9);

    GK_ASSERT_EQ(s2.numV(), s.numV() + 1);
}

// ── Degree elevation in U ─────────────────────────────────────────────────────
GK_TEST(BSplineSurface, DegreeElevateU_PreservesGeometry)
{
    auto s = makeQuadLinear();                   // degree (2,1)
    auto s2 = s.elevateU();                      // degree (3,1)
    GK_ASSERT_EQ(s2.degreeU(), 3);

    for (double u : {0.1, 0.3, 0.5, 0.7, 0.9})
        for (double v : {0.0, 0.5, 1.0})
            expectVec3Near(s.evaluate(u,v).p, s2.evaluate(u,v).p, 1e-8);
}

// ── Degree elevation in V ─────────────────────────────────────────────────────
GK_TEST(BSplineSurface, DegreeElevateV_PreservesGeometry)
{
    auto s = makeBilinear();                     // degree (1,1)
    auto s2 = s.elevateV();                      // degree (1,2)
    GK_ASSERT_EQ(s2.degreeV(), 2);

    for (double u : {0.0, 0.3, 0.7, 1.0})
        for (double v : {0.0, 0.3, 0.7, 1.0})
            expectVec3Near(s.evaluate(u,v).p, s2.evaluate(u,v).p, 1e-8);
}

// ── STEP visual export ─────────────────────────────────────────────────────

GK_TEST(BSplineSurface, STEP_BilinearPatch)
{
    auto s = makeBilinear();
    auto mesh = gk::tessellate(s, 8, 8);
    StepWriter step;
    step.addSurfaceMesh(mesh);
    step.write(stepOutputPath("bspline_bilinear_patch_debug.stp"));
    SUCCEED();
}

GK_TEST(BSplineSurface, STEP_QuadLinearPatch)
{
    auto s = makeQuadLinear();
    auto mesh = gk::tessellate(s, 16, 8);
    StepWriter step;
    step.addSurfaceMesh(mesh);
    step.write(stepOutputPath("bspline_quad_linear_patch_debug.stp"));
    SUCCEED();
}

GK_TEST(BSplineSurface, STEP_BicubicWavePatch)
{
    // 4×4 bicubic patch with a wave-like Z profile
    BSplineSurface::CtrlGrid ctrl(4, std::vector<Vec3>(4));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double u = double(i) / 3.0;
            double v = double(j) / 3.0;
            ctrl[i][j] = Vec3(u * 3.0, v * 3.0,
                              std::sin(u * 3.14159) * std::cos(v * 3.14159));
        }
    auto kU = BSplineSurface::uniformKnots(4, 3);
    auto kV = BSplineSurface::uniformKnots(4, 3);
    BSplineSurface surf(3, 3, kU, kV, ctrl);
    auto mesh = gk::tessellate(surf, 24, 24);
    StepWriter step;
    step.addSurfaceMesh(mesh);
    step.write(stepOutputPath("bspline_bicubic_wave_debug.stp"));
    SUCCEED();
}
