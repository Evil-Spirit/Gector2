#include "GkTest.h"
#include "gk/curve/ICurve.h"
#include "gk/curve/Line.h"
#include "gk/curve/Circle.h"
#include <cmath>

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
