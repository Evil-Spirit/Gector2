#pragma once
// Chapter 5 — Analytic surface: Disc (flat annular disc in polar parameterization).
//
// Parameterization:
//   P(u,v) = center + v*(cos(u)*uRef + sin(u)*vRef)
//   u = azimuthal angle   ∈ [0, 2π]
//   v = radial distance   ∈ [rInner, rOuter]
//
// Natural normal (du × dv) = v*(uRef × vRef) = v*(−axis) direction,
// i.e., pointing in the −axis direction.
// So kForward gives outward normal = −axis (bottom disc of upward cylinder).
// kReversed gives outward normal = +axis (top disc of upward cylinder).

#include "gk/surface/ISurface.h"
#include <cmath>

namespace gk {

class DiscSurface : public ISurface
{
public:
    static constexpr double kPi = 3.14159265358979323846;

    /// @param center   Centre of the disc.
    /// @param uRef     First radial reference direction (in-plane).
    /// @param vRef     Second radial reference direction (in-plane, perp to uRef).
    /// @param rInner   Inner radius (0 for a full disc).
    /// @param rOuter   Outer radius.
    DiscSurface(Vec3 center, Vec3 uRef, Vec3 vRef,
                double rInner, double rOuter) noexcept
        : center_(center)
        , uRef_(uRef.normalized())
        , vRef_(vRef.normalized())
        , rInner_(rInner)
        , rOuter_(rOuter)
    {}

    // ── ISurface ─────────────────────────────────────────────────────────────
    SurfacePoint evaluate(double u, double v) const override
    {
        double cu = std::cos(u), su = std::sin(u);
        Vec3 radial = uRef_ * cu + vRef_ * su;   // unit radial direction
        Vec3 dtang  = vRef_ * cu - uRef_ * su;   // ∂radial/∂u

        SurfacePoint sp;
        sp.p   = center_ + radial * v;
        sp.du  = dtang * v;                      // ∂P/∂u
        sp.dv  = radial;                         // ∂P/∂v

        sp.duu = radial * (-v);                  // ∂²P/∂u² = -v*radial
        sp.duv = dtang;                          // ∂²P/∂u∂v
        sp.dvv = Vec3::zero();                   // ∂²P/∂v² = 0
        return sp;
    }

    SurfaceDomain domain() const override
    {
        return { Interval{0.0, 2.0*kPi}, Interval{rInner_, rOuter_} };
    }

    Vec3 normalAt(double u, double /*v*/) const override
    {
        // du × dv = (dtang*v) × radial = v*(dtang × radial)
        // dtang × radial = (vRef*cu - uRef*su) × (uRef*cu + vRef*su)
        //                = cu*su*(vRef×vRef) + cu²*(vRef×uRef) - su²*(uRef×vRef) - su*cu*(uRef×uRef)
        //                = (cu² + su²)*(vRef×uRef)    [since vRef×vRef=0, uRef×uRef=0]
        //                = vRef × uRef
        //                = -(uRef × vRef)
        // So du×dv points in -(uRef×vRef) direction.
        // We return normalized direction.
        (void)u;
        Vec3 n = uRef_.cross(vRef_); // points in +axis direction conceptually
        return (n * (-1.0)).normalized(); // natural normal is -axis
    }

    void isClosed(bool& cu, bool& cv) const override { cu = true; cv = false; }

    // ── Accessors ─────────────────────────────────────────────────────────────
    const Vec3& center() const noexcept { return center_; }
    const Vec3& uRef()   const noexcept { return uRef_;   }
    const Vec3& vRef()   const noexcept { return vRef_;   }
    double rInner()      const noexcept { return rInner_; }
    double rOuter()      const noexcept { return rOuter_; }

    /// Build an orthonormal frame {uRef, vRef} perpendicular to axis.
    static void buildFrame(const Vec3& axis, Vec3& u, Vec3& v) noexcept
    {
        Vec3 ref = (std::abs(axis.dot(Vec3::unitX())) < 0.9)
                   ? Vec3::unitX() : Vec3::unitY();
        u = (ref - axis * ref.dot(axis)).normalized();
        v = axis.cross(u);
    }

private:
    Vec3   center_, uRef_, vRef_;
    double rInner_, rOuter_;
};

} // namespace gk
