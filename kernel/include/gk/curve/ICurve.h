#pragma once
#include "gk/math/Vec2.h"
#include "gk/math/Vec3.h"
#include "gk/math/Interval.h"
#include <cmath>
#include <vector>

namespace gk {

struct CurvePoint3 {
    Vec3 p;
    Vec3 d1;
    Vec3 d2;
};

struct CurvePoint2 {
    Vec2 p;
    Vec2 d1;
    Vec2 d2;
};

class ICurve3 {
public:
    virtual ~ICurve3() = default;
    virtual CurvePoint3 evaluate(double t) const = 0;
    virtual Interval domain() const = 0;
    virtual bool isClosed() const = 0;

    virtual double approximateLength(int nSamples = 64) const {
        Interval dom = domain();
        double len = 0.0;
        Vec3 prev = evaluate(dom.lo).p;
        for (int i = 1; i <= nSamples; ++i) {
            double t = dom.lo + dom.width() * (double(i) / double(nSamples));
            Vec3 cur = evaluate(t).p;
            len += (cur - prev).norm();
            prev = cur;
        }
        return len;
    }

    Vec3 tangentAt(double t) const {
        return evaluate(t).d1.normalized();
    }

    double curvatureAt(double t) const {
        auto cp = evaluate(t);
        double d1n = cp.d1.norm();
        if (d1n < 1e-14) return 0.0;
        return cp.d1.cross(cp.d2).norm() / (d1n * d1n * d1n);
    }
};

class ICurve2 {
public:
    virtual ~ICurve2() = default;
    virtual CurvePoint2 evaluate(double t) const = 0;
    virtual Interval domain() const = 0;
    virtual bool isClosed() const = 0;

    virtual double approximateLength(int nSamples = 64) const {
        Interval dom = domain();
        double len = 0.0;
        Vec2 prev = evaluate(dom.lo).p;
        for (int i = 1; i <= nSamples; ++i) {
            double t = dom.lo + dom.width() * (double(i) / double(nSamples));
            Vec2 cur = evaluate(t).p;
            len += (cur - prev).norm();
            prev = cur;
        }
        return len;
    }

    Vec2 tangentAt(double t) const {
        return evaluate(t).d1.normalized();
    }

    double curvatureAt(double t) const {
        auto cp = evaluate(t);
        double d1n = cp.d1.norm();
        if (d1n < 1e-14) return 0.0;
        double k = (cp.d1.x * cp.d2.y - cp.d1.y * cp.d2.x) / (d1n * d1n * d1n);
        return std::abs(k);
    }
};

} // namespace gk
