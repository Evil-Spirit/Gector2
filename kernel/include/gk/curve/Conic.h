#pragma once
#include "gk/curve/ICurve.h"
#include <cmath>

namespace gk {

class Hyperbola3 : public ICurve3 {
public:
    Hyperbola3(Vec3 center, double a, double b, Vec3 xAxis, Vec3 yAxis,
               double tMax = 3.0)
        : center_(center), a_(a), b_(b), xAxis_(xAxis), yAxis_(yAxis)
        , tMax_(tMax) {}

    CurvePoint3 evaluate(double t) const override {
        CurvePoint3 cp;
        double ch = std::cosh(t), sh = std::sinh(t);
        cp.p  = center_ + xAxis_ * (a_ * ch) + yAxis_ * (b_ * sh);
        cp.d1 = xAxis_ * (a_ * sh) + yAxis_ * (b_ * ch);
        cp.d2 = xAxis_ * (a_ * ch) + yAxis_ * (b_ * sh);
        return cp;
    }

    Interval domain() const override { return Interval{-tMax_, tMax_}; }
    bool isClosed() const override { return false; }

private:
    Vec3 center_;
    double a_, b_;
    Vec3 xAxis_, yAxis_;
    double tMax_;
};

class Parabola3 : public ICurve3 {
public:
    Parabola3(Vec3 apex, double focalParam, Vec3 xAxis, Vec3 yAxis,
              double tMax = 5.0)
        : apex_(apex), focalParam_(focalParam), xAxis_(xAxis), yAxis_(yAxis)
        , tMax_(tMax) {}

    CurvePoint3 evaluate(double t) const override {
        CurvePoint3 cp;
        double fp2 = 2.0 * focalParam_;
        cp.p  = apex_ + xAxis_ * t + yAxis_ * (t * t / (4.0 * focalParam_));
        cp.d1 = xAxis_ + yAxis_ * (t / fp2);
        cp.d2 = yAxis_ * (1.0 / fp2);
        return cp;
    }

    Interval domain() const override { return Interval{-tMax_, tMax_}; }
    bool isClosed() const override { return false; }

private:
    Vec3 apex_;
    double focalParam_;
    Vec3 xAxis_, yAxis_;
    double tMax_;
};

} // namespace gk
