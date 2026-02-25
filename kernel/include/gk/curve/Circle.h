#pragma once
#include "gk/curve/ICurve.h"
#include <cmath>

namespace gk {

class Circle3 : public ICurve3 {
public:
    Circle3(Vec3 center, double radius, Vec3 xAxis, Vec3 yAxis)
        : center_(center), radius_(radius), xAxis_(xAxis), yAxis_(yAxis)
        , tMin_(0.0), tMax_(2.0 * 3.14159265358979323846) {}

    static Circle3 arc(Vec3 center, double radius, Vec3 xAxis, Vec3 yAxis,
                       double tMin, double tMax) {
        Circle3 c(center, radius, xAxis, yAxis);
        c.tMin_ = tMin;
        c.tMax_ = tMax;
        return c;
    }

    CurvePoint3 evaluate(double t) const override {
        CurvePoint3 cp;
        double ct = std::cos(t), st = std::sin(t);
        cp.p  = center_ + (xAxis_ * ct + yAxis_ * st) * radius_;
        cp.d1 = (-xAxis_ * st + yAxis_ * ct) * radius_;
        cp.d2 = -(xAxis_ * ct + yAxis_ * st) * radius_;
        return cp;
    }

    Interval domain() const override { return Interval{tMin_, tMax_}; }

    bool isClosed() const override {
        constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
        return std::abs((tMax_ - tMin_) - kTwoPi) < 1e-10;
    }

    double approximateLength(int /*nSamples*/ = 64) const override {
        return radius_ * (tMax_ - tMin_);
    }

private:
    Vec3 center_;
    double radius_;
    Vec3 xAxis_, yAxis_;
    double tMin_, tMax_;
};

class Circle2 : public ICurve2 {
public:
    Circle2(Vec2 center, double radius)
        : center_(center), radius_(radius)
        , tMin_(0.0), tMax_(2.0 * 3.14159265358979323846) {}

    static Circle2 arc(Vec2 center, double radius, double tMin, double tMax) {
        Circle2 c(center, radius);
        c.tMin_ = tMin;
        c.tMax_ = tMax;
        return c;
    }

    CurvePoint2 evaluate(double t) const override {
        CurvePoint2 cp;
        double ct = std::cos(t), st = std::sin(t);
        cp.p  = center_ + Vec2(radius_ * ct, radius_ * st);
        cp.d1 = Vec2(-radius_ * st, radius_ * ct);
        cp.d2 = Vec2(-radius_ * ct, -radius_ * st);
        return cp;
    }

    Interval domain() const override { return Interval{tMin_, tMax_}; }

    bool isClosed() const override {
        constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
        return std::abs((tMax_ - tMin_) - kTwoPi) < 1e-10;
    }

    double approximateLength(int /*nSamples*/ = 64) const override {
        return radius_ * (tMax_ - tMin_);
    }

private:
    Vec2 center_;
    double radius_;
    double tMin_, tMax_;
};

} // namespace gk
