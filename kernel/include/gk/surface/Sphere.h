#pragma once
// Iteration 3.2 — Analytic surface: Sphere.
//
// Spherical parameterization (standard longitude / latitude):
//   P(u,v) = center + r*(cos(u)*cos(v), sin(u)*cos(v), sin(v))
//   u = longitude ∈ [0, 2π],  v = latitude ∈ [-π/2, π/2]
//
// Gaussian curvature  K = 1/r²
// Mean curvature      H = 1/r  (unsigned)
// Both principal curvatures κ₁ = κ₂ = 1/r

#include "gk/surface/ISurface.h"
#include <cmath>
#include <utility>

namespace gk {

class Sphere : public ISurface
{
public:
    static constexpr double kPi = 3.14159265358979323846;

    Sphere(Vec3 center, double radius) noexcept
        : center_(center), radius_(radius)
    {}

    // ── ISurface ─────────────────────────────────────────────────────────────
    SurfacePoint evaluate(double u, double v) const override
    {
        double cu = std::cos(u), su = std::sin(u);
        double cv = std::cos(v), sv = std::sin(v);
        double r  = radius_;

        SurfacePoint sp;
        sp.p  = center_ + Vec3{r*cu*cv, r*su*cv, r*sv};
        sp.du = Vec3{-r*su*cv, r*cu*cv, 0.0};
        sp.dv = Vec3{-r*cu*sv, -r*su*sv, r*cv};

        sp.duu = Vec3{-r*cu*cv, -r*su*cv,  0.0};
        sp.duv = Vec3{ r*su*sv, -r*cu*sv,  0.0};
        sp.dvv = Vec3{-r*cu*cv, -r*su*cv, -r*sv};
        return sp;
    }

    SurfaceDomain domain() const override
    {
        return { Interval{0.0, 2.0*kPi}, Interval{-kPi/2.0, kPi/2.0} };
    }

    Vec3 normalAt(double u, double v) const override
    {
        double cu = std::cos(u), su = std::sin(u);
        double cv = std::cos(v), sv = std::sin(v);
        return Vec3{cu*cv, su*cv, sv};          // exact outward unit normal
    }

    void isClosed(bool& cu, bool& cv) const override { cu = true; cv = false; }

    // ── Analytic utilities ────────────────────────────────────────────────────

    /// Map a point on the sphere to (u, v) parameters.
    /// Point is assumed to lie on the sphere; no range check is performed.
    std::pair<double,double> inverseEvaluate(const Vec3& pt) const noexcept
    {
        Vec3   d = (pt - center_) * (1.0 / radius_);
        double u = std::atan2(d.y, d.x);
        if (u < 0.0) u += 2.0*kPi;
        // Clamp to [-1,1] to guard against floating-point rounding before asin.
        double sv = d.z;
        if (sv >  1.0) sv =  1.0;
        if (sv < -1.0) sv = -1.0;
        double v = std::asin(sv);
        return {u, v};
    }

    double gaussianCurvature(double /*u*/, double /*v*/) const noexcept
    {
        return 1.0 / (radius_ * radius_);
    }
    double meanCurvature(double /*u*/, double /*v*/) const noexcept
    {
        return 1.0 / radius_;          // unsigned mean curvature
    }

    // ── Accessors ─────────────────────────────────────────────────────────────
    const Vec3& center() const noexcept { return center_; }
    double       radius() const noexcept { return radius_; }

private:
    Vec3   center_;
    double radius_;
};

} // namespace gk
