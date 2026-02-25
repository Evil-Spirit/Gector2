#pragma once
// Iteration 3.2 — Analytic surface: Plane.
//
// Parameterization: P(u,v) = origin + u*uAxis + v*vAxis
// where uAxis and vAxis are orthonormal.
// Domain: by convention (-1e10, 1e10) in both u and v (effectively infinite).

#include "gk/surface/ISurface.h"
#include <utility>

namespace gk {

class Plane : public ISurface
{
public:
    Plane(Vec3 origin, Vec3 uAxis, Vec3 vAxis) noexcept
        : origin_(origin)
        , uAxis_(uAxis.normalized())
        , vAxis_(vAxis.normalized())
    {}

    // ── Convenient factories ──────────────────────────────────────────────────
    static Plane xyPlane() noexcept
    { return Plane{Vec3::zero(), Vec3::unitX(), Vec3::unitY()}; }
    static Plane xzPlane() noexcept
    { return Plane{Vec3::zero(), Vec3::unitX(), Vec3::unitZ()}; }
    static Plane yzPlane() noexcept
    { return Plane{Vec3::zero(), Vec3::unitY(), Vec3::unitZ()}; }

    // ── ISurface ─────────────────────────────────────────────────────────────
    SurfacePoint evaluate(double u, double v) const override
    {
        SurfacePoint sp;
        sp.p   = origin_ + uAxis_ * u + vAxis_ * v;
        sp.du  = uAxis_;
        sp.dv  = vAxis_;
        sp.duu = Vec3::zero();
        sp.duv = Vec3::zero();
        sp.dvv = Vec3::zero();
        return sp;
    }

    SurfaceDomain domain() const override
    {
        return { Interval{-1e10, 1e10}, Interval{-1e10, 1e10} };
    }

    Vec3 normalAt(double /*u*/, double /*v*/) const override
    {
        return uAxis_.cross(vAxis_).normalized();
    }

    void isClosed(bool& cu, bool& cv) const override { cu = cv = false; }

    // ── Analytic utilities ────────────────────────────────────────────────────

    /// Project a world point onto the plane's (u,v) parameter space.
    std::pair<double,double> inverseEvaluate(const Vec3& pt) const noexcept
    {
        Vec3 d = pt - origin_;
        return { d.dot(uAxis_), d.dot(vAxis_) };
    }

    double gaussianCurvature(double /*u*/, double /*v*/) const noexcept { return 0.0; }
    double meanCurvature(double /*u*/, double /*v*/)     const noexcept { return 0.0; }

    // ── Accessors ─────────────────────────────────────────────────────────────
    const Vec3& origin() const noexcept { return origin_; }
    const Vec3& uAxis()  const noexcept { return uAxis_;  }
    const Vec3& vAxis()  const noexcept { return vAxis_;  }

private:
    Vec3 origin_;
    Vec3 uAxis_;
    Vec3 vAxis_;
};

} // namespace gk
