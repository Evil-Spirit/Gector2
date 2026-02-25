#include "GkTest.h"
#include "gk/surface/NURBSSurface.h"
#include "gk/surface/BSplineSurface.h"
#include <cmath>
#include <vector>

using gk::NURBSSurface;
using gk::BSplineSurface;
using gk::Vec3;

static void expectVec3Near(const Vec3& a, const Vec3& b, double tol = 1e-9)
{
    EXPECT_NEAR(a.x, b.x, tol);
    EXPECT_NEAR(a.y, b.y, tol);
    EXPECT_NEAR(a.z, b.z, tol);
}

// ── NURBS with unit weights == B-spline ───────────────────────────────────────

static NURBSSurface makeUnitWeightNURBS()
{
    // Same as makeBilinear() in BSplineSurface tests.
    NURBSSurface::CtrlGrid pts(2, std::vector<Vec3>(2));
    pts[0][0] = Vec3{0,0,0}; pts[1][0] = Vec3{1,0,0};
    pts[0][1] = Vec3{0,1,0}; pts[1][1] = Vec3{1,1,0};
    return NURBSSurface{1, 1,
                        {0.0,0.0,1.0,1.0},
                        {0.0,0.0,1.0,1.0},
                        pts};       // unit weights supplied by default ctor
}

GK_TEST(NURBSSurface, UnitWeightMatchesBSpline_Corners)
{
    auto n = makeUnitWeightNURBS();
    expectVec3Near(n.evaluate(0.0, 0.0).p, Vec3{0,0,0});
    expectVec3Near(n.evaluate(1.0, 0.0).p, Vec3{1,0,0});
    expectVec3Near(n.evaluate(0.0, 1.0).p, Vec3{0,1,0});
    expectVec3Near(n.evaluate(1.0, 1.0).p, Vec3{1,1,0});
}

GK_TEST(NURBSSurface, UnitWeightMatchesBSpline_Midpoint)
{
    auto n = makeUnitWeightNURBS();
    expectVec3Near(n.evaluate(0.5, 0.5).p, Vec3{0.5, 0.5, 0.0});
}

GK_TEST(NURBSSurface, UnitWeightDerivativesMatchBSpline)
{
    // Build identical surfaces with BSpline and unit-weight NURBS.
    BSplineSurface::CtrlGrid pts(2, std::vector<Vec3>(2));
    pts[0][0] = Vec3{0,0,0}; pts[1][0] = Vec3{1,0,0.5};
    pts[0][1] = Vec3{0,1,0}; pts[1][1] = Vec3{1,1,0.5};
    std::vector<double> kU{0.0,0.0,1.0,1.0}, kV{0.0,0.0,1.0,1.0};

    BSplineSurface bs{1,1,kU,kV,pts};
    NURBSSurface   ns{1,1,kU,kV,pts};   // unit weights

    for (double u : {0.25, 0.5, 0.75})
    {
        for (double v : {0.25, 0.5, 0.75})
        {
            auto sB = bs.evaluate(u,v);
            auto sN = ns.evaluate(u,v);
            expectVec3Near(sB.p,  sN.p,  1e-9);
            expectVec3Near(sB.du, sN.du, 1e-9);
            expectVec3Near(sB.dv, sN.dv, 1e-9);
        }
    }
}

// ── Non-unit weights warp the surface ─────────────────────────────────────────

GK_TEST(NURBSSurface, NonUniformWeights_WarpsSurface)
{
    // Bilinear patch where the (1,0) corner has weight 2.
    // The midpoint should shift toward that corner.
    NURBSSurface::CtrlGrid pts(2, std::vector<Vec3>(2));
    pts[0][0] = Vec3{0,0,0}; pts[1][0] = Vec3{1,0,0};
    pts[0][1] = Vec3{0,1,0}; pts[1][1] = Vec3{1,1,0};
    NURBSSurface::WtGrid wts{{1.0, 1.0}, {2.0, 1.0}};

    NURBSSurface n{1,1,{0.0,0.0,1.0,1.0},{0.0,0.0,1.0,1.0},pts,wts};
    auto sp = n.evaluate(0.5, 0.0);
    // Weighted midpoint should be pulled toward (1,0,0) — x > 0.5
    EXPECT_GT(sp.p.x, 0.5);
}

// ── Derivatives via finite differences ───────────────────────────────────────

GK_TEST(NURBSSurface, Derivatives_FiniteDiff_NonUniformWeights)
{
    NURBSSurface::CtrlGrid pts(2, std::vector<Vec3>(2));
    pts[0][0] = Vec3{0,0,0}; pts[1][0] = Vec3{1,0,0};
    pts[0][1] = Vec3{0,1,0}; pts[1][1] = Vec3{1,1,1};
    NURBSSurface::WtGrid wts{{1.0, 1.0}, {2.0, 0.5}};
    NURBSSurface n{1,1,{0.0,0.0,1.0,1.0},{0.0,0.0,1.0,1.0},pts,wts};

    double h = 1e-5, tol = 1e-4;
    for (double u : {0.25, 0.5, 0.75})
    {
        for (double v : {0.25, 0.5, 0.75})
        {
            auto sp   = n.evaluate(u, v);
            auto du_fd= (n.evaluate(u+h,v).p - n.evaluate(u-h,v).p) * (1.0/(2.0*h));
            auto dv_fd= (n.evaluate(u,v+h).p - n.evaluate(u,v-h).p) * (1.0/(2.0*h));
            EXPECT_NEAR(sp.du.x, du_fd.x, tol);
            EXPECT_NEAR(sp.du.y, du_fd.y, tol);
            EXPECT_NEAR(sp.dv.x, dv_fd.x, tol);
            EXPECT_NEAR(sp.dv.y, dv_fd.y, tol);
        }
    }
}

// ── Domain ────────────────────────────────────────────────────────────────────

GK_TEST(NURBSSurface, Domain)
{
    auto n = makeUnitWeightNURBS();
    auto dom = n.domain();
    EXPECT_NEAR(dom.u.lo, 0.0, 1e-12); EXPECT_NEAR(dom.u.hi, 1.0, 1e-12);
    EXPECT_NEAR(dom.v.lo, 0.0, 1e-12); EXPECT_NEAR(dom.v.hi, 1.0, 1e-12);
}
