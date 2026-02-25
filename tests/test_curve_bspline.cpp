#include "GkTest.h"
#include "SvgWriter.h"
#include "gk/curve/BSplineCurve.h"
#include "gk/curve/NURBSCurve.h"
#include "gk/curve/Circle.h"
#include <cmath>
#include <vector>

using namespace gk;

static constexpr double kPi2 = 3.14159265358979323846;

GK_TEST(BSplineCurve, Degree1Polyline) {
    std::vector<Vec3> ctrl = {
        Vec3(0,0,0), Vec3(1,0,0), Vec3(1,1,0), Vec3(2,1,0)
    };
    auto knots = BSplineCurve3::uniformKnots(4, 1);
    BSplineCurve3 c(1, knots, ctrl);
    auto dom = c.domain();
    auto cp0 = c.evaluate(dom.lo);
    GK_ASSERT_NEAR(cp0.p.x, 0.0, 1e-10);
    GK_ASSERT_NEAR(cp0.p.y, 0.0, 1e-10);
    auto cp1 = c.evaluate(dom.hi);
    GK_ASSERT_NEAR(cp1.p.x, 2.0, 1e-10);
    GK_ASSERT_NEAR(cp1.p.y, 1.0, 1e-10);
}

GK_TEST(BSplineCurve, Degree3Cubic) {
    std::vector<Vec3> ctrl = {
        Vec3(0,0,0), Vec3(1,2,0), Vec3(2,2,0),
        Vec3(3,0,0), Vec3(4,2,0), Vec3(5,0,0)
    };
    auto knots = BSplineCurve3::uniformKnots(6, 3);
    BSplineCurve3 c(3, knots, ctrl);
    auto dom = c.domain();
    for (int i = 0; i <= 20; ++i) {
        double t = dom.lo + dom.width() * (double(i)/20.0);
        auto cp = c.evaluate(t);
        GK_ASSERT_TRUE(std::isfinite(cp.p.x));
        GK_ASSERT_TRUE(std::isfinite(cp.d1.x));
    }
}

GK_TEST(BSplineCurve, Degree2Quadratic2D) {
    std::vector<Vec2> ctrl = {
        Vec2(0,0), Vec2(1,2), Vec2(2,0), Vec2(3,2), Vec2(4,0)
    };
    auto knots = BSplineCurve2::uniformKnots(5, 2);
    BSplineCurve2 c(2, knots, ctrl);
    auto dom = c.domain();
    auto cp0 = c.evaluate(dom.lo);
    GK_ASSERT_NEAR(cp0.p.x, 0.0, 1e-10);
    auto cpMid = c.evaluate(dom.midpoint());
    GK_ASSERT_TRUE(std::isfinite(cpMid.p.x));
}

GK_TEST(BSplineCurve, NURBSUnitWeightsMatchBSpline) {
    std::vector<Vec3> ctrl = {
        Vec3(0,0,0), Vec3(1,2,0), Vec3(2,0,0), Vec3(3,1,0)
    };
    auto knots = BSplineCurve3::uniformKnots(4, 3);
    std::vector<double> weights(4, 1.0);
    BSplineCurve3 bspl(3, knots, ctrl);
    NURBSCurve3 nurbs(3, knots, ctrl, weights);

    auto dom = bspl.domain();
    for (int i = 0; i <= 10; ++i) {
        double t = dom.lo + dom.width() * (double(i)/10.0);
        auto cpB = bspl.evaluate(t);
        auto cpN = nurbs.evaluate(t);
        GK_ASSERT_NEAR(cpB.p.x, cpN.p.x, 1e-10);
        GK_ASSERT_NEAR(cpB.p.y, cpN.p.y, 1e-10);
    }
}

GK_TEST(BSplineCurve, NURBSCircle) {
    const double w1 = std::sqrt(2.0) / 2.0;
    double r = 1.0;
    std::vector<Vec3> ctrl = {
        Vec3( r,  0, 0),
        Vec3( r,  r, 0),
        Vec3( 0,  r, 0),
        Vec3(-r,  r, 0),
        Vec3(-r,  0, 0),
        Vec3(-r, -r, 0),
        Vec3( 0, -r, 0),
        Vec3( r, -r, 0),
        Vec3( r,  0, 0),
    };
    std::vector<double> weights = {1, w1, 1, w1, 1, w1, 1, w1, 1};
    std::vector<double> knots = {
        0, 0, 0,
        0.25, 0.25,
        0.5, 0.5,
        0.75, 0.75,
        1, 1, 1
    };

    NURBSCurve3 nurbs(2, knots, ctrl, weights);
    Circle3 circ(Vec3::zero(), r, Vec3::unitX(), Vec3::unitY());

    struct TestPt { double nt; double ct; };
    std::vector<TestPt> pts = {
        {0.0, 0.0},
        {0.25, kPi2/2.0},
        {0.5, kPi2},
        {0.75, 3.0*kPi2/2.0}
    };
    for (auto& tp : pts) {
        auto cpN = nurbs.evaluate(tp.nt);
        auto cpC = circ.evaluate(tp.ct);
        GK_ASSERT_NEAR(cpN.p.x, cpC.p.x, 1e-8);
        GK_ASSERT_NEAR(cpN.p.y, cpC.p.y, 1e-8);
    }
}

GK_TEST(BSplineCurve, KnotInsertion) {
    std::vector<Vec3> ctrl = {
        Vec3(0,0,0), Vec3(1,2,0), Vec3(2,2,0), Vec3(3,0,0)
    };
    auto knots = BSplineCurve3::uniformKnots(4, 3);
    BSplineCurve3 c(3, knots, ctrl);
    auto dom = c.domain();
    double tInsert = dom.lo + dom.width() * 0.5;
    BSplineCurve3 c2 = c.insertKnot(tInsert);

    for (int i = 0; i <= 20; ++i) {
        double t = dom.lo + dom.width() * (double(i)/20.0);
        auto cp1 = c.evaluate(t);
        auto cp2 = c2.evaluate(t);
        GK_ASSERT_NEAR(cp1.p.x, cp2.p.x, 1e-12);
        GK_ASSERT_NEAR(cp1.p.y, cp2.p.y, 1e-12);
        GK_ASSERT_NEAR(cp1.p.z, cp2.p.z, 1e-12);
    }
}

GK_TEST(BSplineCurve, SVG_Output) {
    SvgWriter svg(900, 700);
    svg.setView(-1, -2, 10, 8);

    {
        std::vector<Vec2> ctrl = {Vec2(0,0), Vec2(1,1), Vec2(2,0), Vec2(3,2)};
        auto knots = BSplineCurve2::uniformKnots(4, 1);
        BSplineCurve2 c(1, knots, ctrl);
        std::vector<Vec2> pts;
        auto dom = c.domain();
        for (int i = 0; i <= 50; ++i) {
            double t = dom.lo + dom.width() * (double(i)/50.0);
            pts.push_back(c.evaluate(t).p);
        }
        svg.addPolyline(pts, "#888888");
    }

    {
        std::vector<Vec2> ctrl = {Vec2(0,2), Vec2(1,4), Vec2(2,2), Vec2(3,4), Vec2(4,2)};
        auto knots = BSplineCurve2::uniformKnots(5, 2);
        BSplineCurve2 c(2, knots, ctrl);
        std::vector<Vec2> pts;
        auto dom = c.domain();
        for (int i = 0; i <= 100; ++i) {
            double t = dom.lo + dom.width() * (double(i)/100.0);
            pts.push_back(c.evaluate(t).p);
        }
        svg.addPolyline(pts, "#0000ff");
    }

    {
        std::vector<Vec2> ctrl = {
            Vec2(5,0), Vec2(5,2), Vec2(6,2), Vec2(6,4),
            Vec2(7,4), Vec2(7,6), Vec2(8,6)
        };
        auto knots = BSplineCurve2::uniformKnots(7, 3);
        BSplineCurve2 c(3, knots, ctrl);
        std::vector<Vec2> pts;
        auto dom = c.domain();
        for (int i = 0; i <= 100; ++i) {
            double t = dom.lo + dom.width() * (double(i)/100.0);
            pts.push_back(c.evaluate(t).p);
        }
        svg.addPolyline(pts, "#ff0000");
        svg.addPolyline(ctrl, "#ffaaaa", 0.8);
    }

    {
        const double w1 = std::sqrt(2.0) / 2.0;
        std::vector<Vec3> ctrl3 = {
            Vec3(4, 3, 0), Vec3(5, 4, 0), Vec3(4, 4, 0),
            Vec3(3, 4, 0), Vec3(3, 3, 0), Vec3(3, 2, 0),
            Vec3(4, 2, 0), Vec3(5, 2, 0), Vec3(4, 3, 0)
        };
        std::vector<double> weights = {1, w1, 1, w1, 1, w1, 1, w1, 1};
        std::vector<double> knots = {0,0,0, 0.25,0.25, 0.5,0.5, 0.75,0.75, 1,1,1};
        NURBSCurve3 nurbs(2, knots, ctrl3, weights);
        std::vector<Vec2> pts;
        auto dom = nurbs.domain();
        for (int i = 0; i <= 100; ++i) {
            double t = dom.lo + dom.width() * (double(i)/100.0);
            auto cp = nurbs.evaluate(t);
            pts.push_back(Vec2(cp.p.x, cp.p.y));
        }
        svg.addPolyline(pts, "#008800");
    }

    svg.write("curve_bspline_debug.svg");
    SUCCEED();
}
