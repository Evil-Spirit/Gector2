#pragma once
#include "gk/curve/ICurve.h"
#include <algorithm>

namespace gk {

class Line3 : public ICurve3 {
public:
    Line3(Vec3 origin, Vec3 dir, double tMin, double tMax)
        : origin_(origin), dir_(dir), tMin_(tMin), tMax_(tMax) {}

    static Line3 fromSegment(Vec3 start, Vec3 end) {
        Vec3 d = end - start;
        return Line3(start, d, 0.0, 1.0);
    }

    static Line3 unbounded(Vec3 pt, Vec3 dir) {
        return Line3(pt, dir, -1e18, 1e18);
    }

    CurvePoint3 evaluate(double t) const override {
        CurvePoint3 cp;
        cp.p  = origin_ + dir_ * t;
        cp.d1 = dir_;
        cp.d2 = Vec3::zero();
        return cp;
    }

    Interval domain() const override { return Interval{tMin_, tMax_}; }
    bool isClosed() const override { return false; }

    double closestParam(const Vec3& pt) const {
        double d2 = dir_.squaredNorm();
        if (d2 < 1e-28) return tMin_;
        return (pt - origin_).dot(dir_) / d2;
    }

    Vec3 closestPoint(const Vec3& pt) const {
        double t = closestParam(pt);
        t = std::max(tMin_, std::min(tMax_, t));
        return evaluate(t).p;
    }

private:
    Vec3 origin_, dir_;
    double tMin_, tMax_;
};

class Line2 : public ICurve2 {
public:
    Line2(Vec2 origin, Vec2 dir, double tMin, double tMax)
        : origin_(origin), dir_(dir), tMin_(tMin), tMax_(tMax) {}

    static Line2 fromSegment(Vec2 start, Vec2 end) {
        Vec2 d = end - start;
        return Line2(start, d, 0.0, 1.0);
    }

    static Line2 unbounded(Vec2 pt, Vec2 dir) {
        return Line2(pt, dir, -1e18, 1e18);
    }

    CurvePoint2 evaluate(double t) const override {
        CurvePoint2 cp;
        cp.p  = origin_ + dir_ * t;
        cp.d1 = dir_;
        cp.d2 = Vec2::zero();
        return cp;
    }

    Interval domain() const override { return Interval{tMin_, tMax_}; }
    bool isClosed() const override { return false; }

    double closestParam(const Vec2& pt) const {
        double d2 = dir_.squaredNorm();
        if (d2 < 1e-28) return tMin_;
        return (pt - origin_).dot(dir_) / d2;
    }

    Vec2 closestPoint(const Vec2& pt) const {
        double t = closestParam(pt);
        t = std::max(tMin_, std::min(tMax_, t));
        return evaluate(t).p;
    }

private:
    Vec2 origin_, dir_;
    double tMin_, tMax_;
};

} // namespace gk
