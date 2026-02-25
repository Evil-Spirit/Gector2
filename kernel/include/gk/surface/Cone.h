#pragma once
// Iteration 3.2 — Analytic surface: Cone.
//
// Parameterization (right circular cone, apex at 'apex', axis = 'axis'):
//   P(u,v) = apex + v*axis + v*tan(halfAngle)*(cos(u)*uRef + sin(u)*vRef)
//   u = azimuthal angle  ∈ [0, 2π]
//   v = signed height from apex ∈ [vMin, vMax]  (vMin >= 0 for a proper cone)
//
// The outward unit normal (independent of v for v > 0):
//   n(u) = cos(α)*(cos(u)*uRef + sin(u)*vRef) - sin(α)*axis
// where α = halfAngle.
//
// Gaussian curvature K = 0  (developable)

#include "gk/surface/ISurface.h"
#include <cmath>
#include <utility>

namespace gk {

class Cone : public ISurface
{
public:
    static constexpr double kPi = 3.14159265358979323846;

    /// @param apex       Apex point.
    /// @param axis       Unit axis direction (cone opens in the positive direction).
    /// @param halfAngle  Half-opening angle in radians (0 < halfAngle < π/2).
    /// @param vMin/vMax  Height range (vMin ≥ 0 to avoid the apex singularity).
    Cone(Vec3 apex, Vec3 axis, double halfAngle,
         double vMin = 0.0, double vMax = 1.0) noexcept
        : apex_(apex)
        , axis_(axis.normalized())
        , halfAngle_(halfAngle)
        , vMin_(vMin)
        , vMax_(vMax)
    {
        buildFrame(axis_, uRef_, vRef_);
        tanA_ = std::tan(halfAngle_);
        cosA_ = std::cos(halfAngle_);
        sinA_ = std::sin(halfAngle_);
    }

    // ── ISurface ─────────────────────────────────────────────────────────────
    SurfacePoint evaluate(double u, double v) const override
    {
        double cu = std::cos(u), su = std::sin(u);
        Vec3 radial = uRef_ * cu + vRef_ * su;          // unit radial direction
        Vec3 dtang  = vRef_ * cu - uRef_ * su;          // ∂radial/∂u

        double r = v * tanA_;                            // radius at height v

        SurfacePoint sp;
        sp.p  = apex_ + axis_ * v + radial * r;
        sp.du = dtang * r;                               // ∂P/∂u = v*tanA*∂radial/∂u
        sp.dv = axis_ + radial * tanA_;                  // ∂P/∂v = axis + tanA*radial

        // ∂²P/∂u² = -r*radial
        sp.duu = radial * (-r);
        // ∂²P/∂u∂v = tanA * ∂radial/∂u
        sp.duv = dtang * tanA_;
        // ∂²P/∂v² = 0
        sp.dvv = Vec3::zero();
        return sp;
    }

    SurfaceDomain domain() const override
    {
        return { Interval{0.0, 2.0*kPi}, Interval{vMin_, vMax_} };
    }

    Vec3 normalAt(double u, double /*v*/) const override
    {
        // Outward normal is independent of v.
        Vec3 radial = uRef_ * std::cos(u) + vRef_ * std::sin(u);
        return (radial * cosA_ - axis_ * sinA_);
    }

    void isClosed(bool& cu, bool& cv) const override { cu = true; cv = false; }

    // ── Analytic utilities ────────────────────────────────────────────────────

    std::pair<double,double> inverseEvaluate(const Vec3& pt) const noexcept
    {
        Vec3   d = pt - apex_;
        double v = d.dot(axis_);
        Vec3   lateral = d - axis_ * v;
        double u = std::atan2(lateral.dot(vRef_), lateral.dot(uRef_));
        if (u < 0.0) u += 2.0 * kPi;
        return {u, v};
    }

    double gaussianCurvature(double /*u*/, double /*v*/) const noexcept { return 0.0; }

    // ── Accessors ─────────────────────────────────────────────────────────────
    const Vec3& apex()       const noexcept { return apex_;      }
    const Vec3& axis()       const noexcept { return axis_;      }
    double       halfAngle() const noexcept { return halfAngle_; }

private:
    Vec3   apex_, axis_, uRef_, vRef_;
    double halfAngle_, tanA_, cosA_, sinA_;
    double vMin_, vMax_;

    static void buildFrame(const Vec3& axis, Vec3& u, Vec3& v) noexcept
    {
        Vec3 ref = (std::abs(axis.dot(Vec3::unitX())) < 0.9)
                   ? Vec3::unitX() : Vec3::unitY();
        u = (ref - axis * ref.dot(axis)).normalized();
        v = axis.cross(u);
    }
};

} // namespace gk
