#pragma once
// Iteration 3.2 — Analytic surface: Cylinder.
//
// Parameterization:
//   P(u,v) = origin + v*axis + r*(cos(u)*uRef + sin(u)*vRef)
//   u = azimuthal angle  ∈ [0, 2π]
//   v = height along axis ∈ [vMin, vMax]
//
// The orthonormal frame {uRef, vRef, axis} is built automatically from 'axis'.
//
// Principal curvatures:  κ₁ = 1/r (circumferential),  κ₂ = 0 (axial)
// Gaussian curvature K = 0   (developable surface)
// Mean curvature     H = 1/(2r)

#include "gk/surface/ISurface.h"
#include <cmath>
#include <utility>

namespace gk {

class Cylinder : public ISurface
{
public:
    static constexpr double kPi = 3.14159265358979323846;

    /// @param origin   Point on the axis (used as v=0 reference).
    /// @param axis     Unit direction of the cylinder axis.
    /// @param radius   Cylinder radius.
    /// @param vMin/vMax  Height range along the axis.
    Cylinder(Vec3 origin, Vec3 axis, double radius,
             double vMin = 0.0, double vMax = 1.0) noexcept
        : origin_(origin)
        , axis_(axis.normalized())
        , radius_(radius)
        , vMin_(vMin)
        , vMax_(vMax)
    {
        buildFrame(axis_, uRef_, vRef_);
    }

    /// Constructor that accepts an explicit x-reference direction (uRef).
    /// When xRef is non-zero and perpendicular to axis, it is used directly.
    Cylinder(Vec3 origin, Vec3 axis, double radius,
             Vec3 xRef,
             double vMin = 0.0, double vMax = 1.0) noexcept
        : origin_(origin)
        , axis_(axis.normalized())
        , radius_(radius)
        , vMin_(vMin)
        , vMax_(vMax)
    {
        buildFrame(axis_, uRef_, vRef_);
        if (xRef.squaredNorm() > 1e-20) {
            Vec3 x = xRef - axis_ * axis_.dot(xRef);
            if (x.squaredNorm() > 1e-20) {
                uRef_ = x.normalized();
                vRef_ = axis_.cross(uRef_);
            }
        }
    }

    // ── ISurface ─────────────────────────────────────────────────────────────
    SurfacePoint evaluate(double u, double v) const override
    {
        double cu = std::cos(u), su = std::sin(u);
        double r  = radius_;

        Vec3 radial = uRef_ * cu + vRef_ * su;
        Vec3 tang   = vRef_ * cu - uRef_ * su;   // ∂radial/∂u

        SurfacePoint sp;
        sp.p  = origin_ + axis_ * v + radial * r;
        sp.du = tang * r;
        sp.dv = axis_;

        sp.duu = radial * (-r);               // ∂²/∂u² = -r*radial
        sp.duv = Vec3::zero();                // axis constant w.r.t. u
        sp.dvv = Vec3::zero();                // axis constant w.r.t. v
        return sp;
    }

    SurfaceDomain domain() const override
    {
        return { Interval{0.0, 2.0*kPi}, Interval{vMin_, vMax_} };
    }

    Vec3 normalAt(double u, double /*v*/) const override
    {
        // Outward radial normal, independent of v.
        return uRef_ * std::cos(u) + vRef_ * std::sin(u);
    }

    void isClosed(bool& cu, bool& cv) const override { cu = true; cv = false; }

    // ── Analytic utilities ────────────────────────────────────────────────────

    std::pair<double,double> inverseEvaluate(const Vec3& pt) const noexcept
    {
        Vec3   d    = pt - origin_;
        double v    = d.dot(axis_);
        Vec3   proj = d - axis_ * v;
        double u    = std::atan2(proj.dot(vRef_), proj.dot(uRef_));
        if (u < 0.0) u += 2.0 * kPi;
        return {u, v};
    }

    double gaussianCurvature(double /*u*/, double /*v*/) const noexcept { return 0.0; }
    double meanCurvature(double /*u*/, double /*v*/) const noexcept { return 0.5 / radius_; }

    // ── Accessors ─────────────────────────────────────────────────────────────
    const Vec3& origin() const noexcept { return origin_; }
    const Vec3& axis()   const noexcept { return axis_;   }
    const Vec3& uRef()   const noexcept { return uRef_;   }
    double       radius() const noexcept { return radius_; }

private:
    Vec3   origin_, axis_, uRef_, vRef_;
    double radius_, vMin_, vMax_;

    static void buildFrame(const Vec3& axis, Vec3& u, Vec3& v) noexcept
    {
        Vec3 ref = (std::abs(axis.dot(Vec3::unitX())) < 0.9)
                   ? Vec3::unitX() : Vec3::unitY();
        u = (ref - axis * ref.dot(axis)).normalized();
        v = axis.cross(u);
    }
};

} // namespace gk
