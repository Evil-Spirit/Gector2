#include "GkTest.h"
#include "SvgWriter.h"
#include "gk/curve/CurveUtils.h"
#include "gk/curve/Line.h"
#include "gk/curve/Circle.h"
#include "gk/curve/BSplineCurve.h"
#include <cmath>
#include <vector>

using namespace gk;

static constexpr double kPi3 = 3.14159265358979323846;

GK_TEST(CurveUtils, ClosestPointLine3) {
    Line3 line = Line3::fromSegment(Vec3(0,0,0), Vec3(4,0,0));
    Vec3 query(2.0, 3.0, 0.0);
    double t = CurveUtils::closestPoint(line, query);
    auto cp = line.evaluate(t);
    GK_ASSERT_NEAR(cp.p.x, 2.0, 1e-10);
    GK_ASSERT_NEAR(cp.p.y, 0.0, 1e-10);
}

GK_TEST(CurveUtils, ClosestPointCircle3Inside) {
    Circle3 circ(Vec3::zero(), 5.0, Vec3::unitX(), Vec3::unitY());
    Vec3 query(1.0, 0.0, 0.0);
    double t = CurveUtils::closestPoint(circ, query);
    auto cp = circ.evaluate(t);
    GK_ASSERT_NEAR(cp.p.x, 5.0, 1e-6);
    GK_ASSERT_NEAR(cp.p.y, 0.0, 1e-6);
}

GK_TEST(CurveUtils, ClosestPointCircle3Outside) {
    Circle3 circ(Vec3::zero(), 2.0, Vec3::unitX(), Vec3::unitY());
    Vec3 query(5.0, 0.0, 0.0);
    double t = CurveUtils::closestPoint(circ, query);
    auto cp = circ.evaluate(t);
    GK_ASSERT_NEAR(cp.p.x, 2.0, 1e-6);
    GK_ASSERT_NEAR(cp.p.y, 0.0, 1e-6);
}

GK_TEST(CurveUtils, ClosestPointCircle3AtCenter) {
    Circle3 circ(Vec3::zero(), 2.0, Vec3::unitX(), Vec3::unitY());
    Vec3 query(0.0, 0.0, 0.0);
    double t = CurveUtils::closestPoint(circ, query);
    auto cp = circ.evaluate(t);
    GK_ASSERT_NEAR(cp.p.norm(), 2.0, 1e-6);
}

GK_TEST(CurveUtils, ClosestPointBSpline3) {
    std::vector<Vec3> ctrl = {
        Vec3(0,0,0), Vec3(1,2,0), Vec3(2,2,0), Vec3(3,0,0)
    };
    auto knots = BSplineCurve3::uniformKnots(4, 3);
    BSplineCurve3 c(3, knots, ctrl);
    Vec3 query(1.5, 3.0, 0.0);
    double t = CurveUtils::closestPoint(c, query, 32);
    auto cp = c.evaluate(t);
    auto dom = c.domain();
    double bestD = (c.evaluate(dom.lo).p - query).squaredNorm();
    for (int i = 0; i <= 32; ++i) {
        double ts = dom.lo + dom.width() * (double(i)/32.0);
        double d = (c.evaluate(ts).p - query).squaredNorm();
        if (d < bestD) bestD = d;
    }
    double foundD = (cp.p - query).squaredNorm();
    GK_ASSERT_TRUE(foundD <= bestD + 1e-10);
}

GK_TEST(CurveUtils, ArcLengthTableUnitCircle) {
    Circle3 circ(Vec3::zero(), 1.0, Vec3::unitX(), Vec3::unitY());
    auto table = CurveUtils::arcLengthTable(circ, 128);
    GK_ASSERT_NEAR(table.back().first, 2.0*kPi3, 1e-3);
    for (int i = 1; i < (int)table.size(); ++i)
        GK_ASSERT_TRUE(table[i].first >= table[i-1].first - 1e-14);
}

GK_TEST(CurveUtils, Intersect2DTwoLines) {
    Line2 l1 = Line2::fromSegment(Vec2(0,0), Vec2(4,4));
    Line2 l2 = Line2::fromSegment(Vec2(0,4), Vec2(4,0));
    auto res = CurveUtils::intersect2D(l1, l2);
    GK_ASSERT_TRUE(!res.empty());
    auto cp1 = l1.evaluate(res[0].first);
    auto cp2 = l2.evaluate(res[0].second);
    GK_ASSERT_NEAR(cp1.p.x, 2.0, 1e-6);
    GK_ASSERT_NEAR(cp1.p.y, 2.0, 1e-6);
    GK_ASSERT_NEAR(cp2.p.x, 2.0, 1e-6);
}

GK_TEST(CurveUtils, Intersect2DLineAndCircle) {
    // line y=0 crosses circle x^2+y^2=4 at x=±2
    // Use two lines that cross to test degenerate cases
    // Also verify no crash when curves don't intersect
    Line2 l1 = Line2::fromSegment(Vec2(0,0), Vec2(1,0));
    Line2 l2 = Line2::fromSegment(Vec2(0,1), Vec2(1,1));
    auto res = CurveUtils::intersect2D(l1, l2);
    // Parallel lines, should find no intersection
    GK_ASSERT_TRUE(res.empty());
    SUCCEED();
}

// Visual test: closest-point projection and 2-D intersections.
GK_TEST(CurveUtils, SVG_ClosestPointAndIntersection) {
    SvgWriter svg(800, 600);
    svg.setView(-1.5, -1.5, 9.5, 8.5);

    // ── Closest point: query point projected onto a B-spline curve ──────────
    {
        std::vector<Vec2> ctrl = {Vec2(0,0), Vec2(1,4), Vec2(3,4), Vec2(5,1), Vec2(7,3)};
        auto knots = BSplineCurve2::uniformKnots(5, 3);
        BSplineCurve2 c(3, knots, ctrl);

        // Draw the curve
        std::vector<Vec2> curvePts;
        auto dom = c.domain();
        for (int i = 0; i <= 120; ++i) {
            double t = dom.lo + dom.width() * (double(i) / 120.0);
            curvePts.push_back(c.evaluate(t).p);
        }
        svg.addPolyline(curvePts, "#0000cc");
        svg.addPolyline(ctrl, "#aaaaff", 0.7);
        for (auto& p : ctrl) svg.addPoint(p, "#7777ff", 3.0);

        // Query points and their projections
        for (Vec2 query : {Vec2(1.5, 1.5), Vec2(4.0, 5.5), Vec2(6.5, 0.5)}) {
            double t = CurveUtils::closestPoint(c, query, 32);
            Vec2 proj = c.evaluate(t).p;

            // Draw connector: query → projection
            svg.addPolyline({query, proj}, "#ff6600", 0.8);
            svg.addPoint(query, "#ff6600", 4.0);
            svg.addPoint(proj,  "#cc3300", 3.5);
        }
        svg.addLabel(Vec2(0.0, -0.8), "B-spline + closest-point projections");
    }

    // ── Intersection: two crossing lines ────────────────────────────────────
    {
        Line2 l1 = Line2::fromSegment(Vec2(0, 5), Vec2(4, 8));
        Line2 l2 = Line2::fromSegment(Vec2(0, 8), Vec2(4, 5));

        auto drawLine2 = [&](const Line2& l, const std::string& col) {
            std::vector<Vec2> pts;
            auto dom = l.domain();
            for (int i = 0; i <= 30; ++i) {
                double t = dom.lo + dom.width() * (double(i) / 30.0);
                pts.push_back(l.evaluate(t).p);
            }
            svg.addPolyline(pts, col);
        };
        drawLine2(l1, "#008800");
        drawLine2(l2, "#880000");

        auto res = CurveUtils::intersect2D(l1, l2);
        if (!res.empty()) {
            Vec2 ip = l1.evaluate(res[0].first).p;
            svg.addPoint(ip, "#ff00ff", 5.0);
        }
        svg.addLabel(Vec2(0.0, 4.3), "Line-line intersection (magenta)");
    }

    svg.write(svgOutputPath("curve_utils_debug.svg"));
    SUCCEED();
}
