#pragma once
#include "gk/curve/ICurve.h"
#include <cmath>

namespace gk {

class Ellipse3 : public ICurve3 {
public:
    Ellipse3(Vec3 center, double a, double b, Vec3 xAxis, Vec3 yAxis)
        : center_(center), a_(a), b_(b), xAxis_(xAxis), yAxis_(yAxis) {}

    CurvePoint3 evaluate(double t) const override {
        CurvePoint3 cp;
        double ct = std::cos(t), st = std::sin(t);
        cp.p  = center_ + xAxis_ * (a_ * ct) + yAxis_ * (b_ * st);
        cp.d1 = xAxis_ * (-a_ * st) + yAxis_ * (b_ * ct);
        cp.d2 = xAxis_ * (-a_ * ct) + yAxis_ * (-b_ * st);
        return cp;
    }

    Interval domain() const override {
        return Interval{0.0, 2.0 * 3.14159265358979323846};
    }

    bool isClosed() const override { return true; }

private:
    Vec3 center_;
    double a_, b_;
    Vec3 xAxis_, yAxis_;
};

class Ellipse2 : public ICurve2 {
public:
    Ellipse2(Vec2 center, double a, double b)
        : center_(center), a_(a), b_(b) {}

    CurvePoint2 evaluate(double t) const override {
        CurvePoint2 cp;
        double ct = std::cos(t), st = std::sin(t);
        cp.p  = center_ + Vec2(a_ * ct, b_ * st);
        cp.d1 = Vec2(-a_ * st, b_ * ct);
        cp.d2 = Vec2(-a_ * ct, -b_ * st);
        return cp;
    }

    Interval domain() const override {
        return Interval{0.0, 2.0 * 3.14159265358979323846};
    }

    bool isClosed() const override { return true; }

private:
    Vec2 center_;
    double a_, b_;
};

} // namespace gk
