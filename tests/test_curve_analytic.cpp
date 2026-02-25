#include "GkTest.h"
#include "SvgWriter.h"
#include "gk/curve/Line.h"
#include "gk/curve/Circle.h"
#include "gk/curve/Ellipse.h"
#include "gk/curve/Conic.h"
#include <cmath>
#include <vector>

using namespace gk;

static constexpr double kPi = 3.14159265358979323846;

GK_TEST(CurveAnalytic, Line3Eval) {
    Line3 line = Line3::fromSegment(Vec3(0,0,0), Vec3(2,0,0));
    auto cp0 = line.evaluate(0.0);
    auto cp1 = line.evaluate(1.0);
    GK_ASSERT_NEAR(cp0.p.x, 0.0, 1e-12);
    GK_ASSERT_NEAR(cp1.p.x, 2.0, 1e-12);
    GK_ASSERT_NEAR(line.curvatureAt(0.5), 0.0, 1e-12);
}

GK_TEST(CurveAnalytic, Line3ClosestPoint) {
    Line3 line = Line3::fromSegment(Vec3(0,0,0), Vec3(2,0,0));
    Vec3 closest = line.closestPoint(Vec3(1.0, 1.0, 0.0));
    GK_ASSERT_NEAR(closest.x, 1.0, 1e-10);
    GK_ASSERT_NEAR(closest.y, 0.0, 1e-10);
}

GK_TEST(CurveAnalytic, Line3Length) {
    Line3 line = Line3::fromSegment(Vec3(0,0,0), Vec3(3,4,0));
    double len = line.approximateLength(64);
    GK_ASSERT_NEAR(len, 5.0, 1e-6);
}

GK_TEST(CurveAnalytic, Line2Eval) {
    Line2 line = Line2::fromSegment(Vec2(0,0), Vec2(1,1));
    auto cp0 = line.evaluate(0.0);
    auto cp1 = line.evaluate(1.0);
    GK_ASSERT_NEAR(cp0.p.x, 0.0, 1e-12);
    GK_ASSERT_NEAR(cp1.p.x, 1.0, 1e-12);
    GK_ASSERT_NEAR(cp1.p.y, 1.0, 1e-12);
    GK_ASSERT_NEAR(line.curvatureAt(0.5), 0.0, 1e-12);
}

GK_TEST(CurveAnalytic, Line2ClosestPoint) {
    Line2 line = Line2::fromSegment(Vec2(0,0), Vec2(4,0));
    Vec2 closest = line.closestPoint(Vec2(2.0, 3.0));
    GK_ASSERT_NEAR(closest.x, 2.0, 1e-10);
    GK_ASSERT_NEAR(closest.y, 0.0, 1e-10);
}

GK_TEST(CurveAnalytic, Circle3KnownPositions) {
    double r = 2.0;
    Circle3 circ(Vec3::zero(), r, Vec3::unitX(), Vec3::unitY());
    auto cp0 = circ.evaluate(0.0);
    GK_ASSERT_NEAR(cp0.p.x, r, 1e-12);
    GK_ASSERT_NEAR(cp0.p.y, 0.0, 1e-12);
    auto cpHalf = circ.evaluate(kPi/2.0);
    GK_ASSERT_NEAR(cpHalf.p.x, 0.0, 1e-12);
    GK_ASSERT_NEAR(cpHalf.p.y, r, 1e-12);
    auto cpPi = circ.evaluate(kPi);
    GK_ASSERT_NEAR(cpPi.p.x, -r, 1e-12);
    auto cp3Half = circ.evaluate(3.0*kPi/2.0);
    GK_ASSERT_NEAR(cp3Half.p.y, -r, 1e-12);
}

GK_TEST(CurveAnalytic, Circle3DerivPerpendicular) {
    Circle3 circ(Vec3::zero(), 1.5, Vec3::unitX(), Vec3::unitY());
    auto cp = circ.evaluate(0.7);
    GK_ASSERT_NEAR(cp.d1.dot(cp.d2), 0.0, 1e-12);
}

GK_TEST(CurveAnalytic, Circle3Length) {
    double r = 2.0;
    Circle3 circ(Vec3::zero(), r, Vec3::unitX(), Vec3::unitY());
    GK_ASSERT_NEAR(circ.approximateLength(), 2.0*kPi*r, 1e-10);
}

GK_TEST(CurveAnalytic, Circle3Curvature) {
    double r = 3.5;
    Circle3 circ(Vec3::zero(), r, Vec3::unitX(), Vec3::unitY());
    GK_ASSERT_NEAR(circ.curvatureAt(1.0), 1.0/r, 1e-10);
}

GK_TEST(CurveAnalytic, Circle3IsClosed) {
    Circle3 circ(Vec3::zero(), 1.0, Vec3::unitX(), Vec3::unitY());
    GK_ASSERT_TRUE(circ.isClosed());
    Circle3 arc = Circle3::arc(Vec3::zero(), 1.0, Vec3::unitX(), Vec3::unitY(), 0.0, kPi);
    GK_ASSERT_FALSE(arc.isClosed());
}

GK_TEST(CurveAnalytic, Circle2KnownPositions) {
    Circle2 circ(Vec2(1.0, 2.0), 3.0);
    auto cp0 = circ.evaluate(0.0);
    GK_ASSERT_NEAR(cp0.p.x, 4.0, 1e-12);
    GK_ASSERT_NEAR(cp0.p.y, 2.0, 1e-12);
}

GK_TEST(CurveAnalytic, Ellipse3AtZero) {
    Vec3 center(1,2,3);
    double a = 4.0, b = 2.0;
    Ellipse3 ell(center, a, b, Vec3::unitX(), Vec3::unitY());
    auto cp0 = ell.evaluate(0.0);
    GK_ASSERT_NEAR(cp0.p.x, center.x + a, 1e-12);
    GK_ASSERT_NEAR(cp0.p.y, center.y, 1e-12);
}

GK_TEST(CurveAnalytic, Ellipse3AtHalfPi) {
    Vec3 center(0,0,0);
    double a = 4.0, b = 2.0;
    Ellipse3 ell(center, a, b, Vec3::unitX(), Vec3::unitY());
    auto cpHalf = ell.evaluate(kPi/2.0);
    GK_ASSERT_NEAR(cpHalf.p.x, 0.0, 1e-12);
    GK_ASSERT_NEAR(cpHalf.p.y, b, 1e-12);
}

GK_TEST(CurveAnalytic, Ellipse2AtZero) {
    Ellipse2 ell(Vec2(0,0), 3.0, 2.0);
    auto cp0 = ell.evaluate(0.0);
    GK_ASSERT_NEAR(cp0.p.x, 3.0, 1e-12);
    GK_ASSERT_NEAR(cp0.p.y, 0.0, 1e-12);
}

GK_TEST(CurveAnalytic, Hyperbola3AtZero) {
    Hyperbola3 hyp(Vec3::zero(), 2.0, 1.0, Vec3::unitX(), Vec3::unitY());
    auto cp0 = hyp.evaluate(0.0);
    GK_ASSERT_NEAR(cp0.p.x, 2.0, 1e-12);
    GK_ASSERT_NEAR(cp0.p.y, 0.0, 1e-12);
}

GK_TEST(CurveAnalytic, Hyperbola3Derivatives) {
    Hyperbola3 hyp(Vec3::zero(), 2.0, 1.0, Vec3::unitX(), Vec3::unitY());
    auto cp = hyp.evaluate(1.0);
    GK_ASSERT_NEAR(cp.d1.x, 2.0*std::sinh(1.0), 1e-10);
    GK_ASSERT_NEAR(cp.d1.y, 1.0*std::cosh(1.0), 1e-10);
}

GK_TEST(CurveAnalytic, Parabola3AtZero) {
    Parabola3 par(Vec3(1,2,3), 1.0, Vec3::unitX(), Vec3::unitY());
    auto cp0 = par.evaluate(0.0);
    GK_ASSERT_NEAR(cp0.p.x, 1.0, 1e-12);
    GK_ASSERT_NEAR(cp0.p.y, 2.0, 1e-12);
    GK_ASSERT_NEAR(cp0.p.z, 3.0, 1e-12);
}

GK_TEST(CurveAnalytic, Parabola3Derivatives) {
    Parabola3 par(Vec3::zero(), 1.0, Vec3::unitX(), Vec3::unitY());
    auto cp = par.evaluate(2.0);
    GK_ASSERT_NEAR(cp.d1.x, 1.0, 1e-12);
    GK_ASSERT_NEAR(cp.d1.y, 1.0, 1e-12);
    GK_ASSERT_NEAR(cp.d2.x, 0.0, 1e-12);
    GK_ASSERT_NEAR(cp.d2.y, 0.5, 1e-12);
}

GK_TEST(CurveAnalytic, SVG_Output) {
    SvgWriter svg(800, 600);
    svg.setView(-2, -2, 16, 14);

    {
        Line2 line = Line2::fromSegment(Vec2(0,0), Vec2(5,3));
        std::vector<gk::Vec2> pts;
        auto dom = line.domain();
        for (int i = 0; i <= 100; ++i) {
            double t = dom.lo + dom.width() * (double(i)/100.0);
            pts.push_back(line.evaluate(t).p);
        }
        svg.addPolyline(pts, "#000000");
    }

    {
        Circle2 circ(Vec2(8,5), 3.0);
        std::vector<gk::Vec2> pts;
        auto dom = circ.domain();
        for (int i = 0; i <= 100; ++i) {
            double t = dom.lo + dom.width() * (double(i)/100.0);
            pts.push_back(circ.evaluate(t).p);
        }
        svg.addPolyline(pts, "#0000ff");
        for (double t : {0.0, kPi/4.0, kPi/2.0}) {
            auto cp = circ.evaluate(t);
            svg.addTangent(cp.p, cp.d1, 1.0, "#0000aa");
        }
    }

    {
        Ellipse2 ell(Vec2(0,8), 4.0, 2.0);
        std::vector<gk::Vec2> pts;
        auto dom = ell.domain();
        for (int i = 0; i <= 100; ++i) {
            double t = dom.lo + dom.width() * (double(i)/100.0);
            pts.push_back(ell.evaluate(t).p);
        }
        svg.addPolyline(pts, "#008800");
    }

    {
        svg.addPoint(Vec2(5,5), "#ff0000", 2.0);
    }

    svg.write("curve_analytic_debug.svg");
    SUCCEED();
}
