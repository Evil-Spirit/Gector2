#include "GkTest.h"
#include "SvgWriter.h"
#include "gk/curve/ICurve.h"
#include "gk/curve/Line.h"
#include "gk/curve/Circle.h"
#include <cmath>
#include <vector>

using namespace gk;

GK_TEST(CurveIface, Line3EvaluateDomain) {
    Line3 line = Line3::fromSegment(Vec3(0,0,0), Vec3(1,0,0));
    auto cp0 = line.evaluate(0.0);
    GK_ASSERT_NEAR(cp0.p.x, 0.0, 1e-12);
    GK_ASSERT_NEAR(cp0.p.y, 0.0, 1e-12);
    auto cp1 = line.evaluate(1.0);
    GK_ASSERT_NEAR(cp1.p.x, 1.0, 1e-12);
    auto dom = line.domain();
    GK_ASSERT_NEAR(dom.lo, 0.0, 1e-12);
    GK_ASSERT_NEAR(dom.hi, 1.0, 1e-12);
    GK_ASSERT_FALSE(line.isClosed());
}

GK_TEST(CurveIface, TangentNormalized) {
    Line3 line = Line3::fromSegment(Vec3(0,0,0), Vec3(3,4,0));
    auto tang = line.tangentAt(0.5);
    GK_ASSERT_NEAR(tang.norm(), 1.0, 1e-12);
}

GK_TEST(CurveIface, ApproximateLength) {
    Line3 line = Line3::fromSegment(Vec3(0,0,0), Vec3(1,0,0));
    double len = line.approximateLength(64);
    GK_ASSERT_NEAR(len, 1.0, 1e-10);
}

GK_TEST(CurveIface, CurvatureCircle) {
    double r = 3.0;
    Circle3 circ(Vec3::zero(), r, Vec3::unitX(), Vec3::unitY());
    double kappa = circ.curvatureAt(0.0);
    GK_ASSERT_NEAR(kappa, 1.0/r, 1e-10);
}

GK_TEST(CurveIface, Line3ZeroCurvature) {
    Line3 line = Line3::fromSegment(Vec3(0,0,0), Vec3(1,1,1));
    double kappa = line.curvatureAt(0.5);
    GK_ASSERT_NEAR(kappa, 0.0, 1e-12);
}

// Visual test: line + circle with tangent arrows showing curvature interface.
GK_TEST(CurveIface, SVG_LineAndCircle) {
    static constexpr double kPi = 3.14159265358979323846;
    SvgWriter svg(700, 500);
    svg.setView(-1.5, -5.5, 12.5, 6.5);

    // Line segment drawn as 2-D projection (XY plane)
    {
        Line2 line = Line2::fromSegment(Vec2(0, 0), Vec2(10, 0));
        std::vector<Vec2> pts;
        auto dom = line.domain();
        for (int i = 0; i <= 60; ++i) {
            double t = dom.lo + dom.width() * (double(i) / 60.0);
            pts.push_back(line.evaluate(t).p);
        }
        svg.addPolyline(pts, "#333333");
        // tangent at start, mid, end
        for (double t : {dom.lo, dom.midpoint(), dom.hi}) {
            auto cp = line.evaluate(t);
            svg.addTangent(cp.p, cp.d1, 0.8, "#888888");
        }
        svg.addLabel(Vec2(4.5, -0.7), "Line2 (kappa=0)");
    }

    // Full circle (r=2) with tangent arrows at 4 quadrant points
    {
        Circle2 circ(Vec2(5, 2), 2.0);
        std::vector<Vec2> pts;
        auto dom = circ.domain();
        for (int i = 0; i <= 100; ++i) {
            double t = dom.lo + dom.width() * (double(i) / 100.0);
            pts.push_back(circ.evaluate(t).p);
        }
        svg.addPolyline(pts, "#0055cc");
        for (double t : {0.0, kPi / 2.0, kPi, 3.0 * kPi / 2.0}) {
            auto cp = circ.evaluate(t);
            svg.addTangent(cp.p, cp.d1, 0.9, "#0022aa");
        }
        svg.addLabel(Vec2(3.8, -0.4), "Circle2 r=2 (kappa=0.5)");
    }

    // Small circle (r=0.5) to illustrate higher curvature
    {
        Circle2 small(Vec2(0.5, 3), 0.5);
        std::vector<Vec2> pts;
        auto dom = small.domain();
        for (int i = 0; i <= 80; ++i) {
            double t = dom.lo + dom.width() * (double(i) / 80.0);
            pts.push_back(small.evaluate(t).p);
        }
        svg.addPolyline(pts, "#cc5500");
        svg.addLabel(Vec2(-1.0, 3.6), "r=0.5");
    }

    svg.write(svgOutputPath("curve_iface_debug.svg"));
    SUCCEED();
}
