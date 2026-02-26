#pragma once
// Iteration 3.2 — Analytic surface: Torus.
//
// Parameterization (centre C, axis A, major radius R, minor radius r):
//   P(u,v) = C + (R + r*cos(v))*(cos(u)*uRef + sin(u)*vRef) + r*sin(v)*A
//   u = major (longitudinal) angle  ∈ [0, 2π]
//   v = minor (meridional)  angle  ∈ [0, 2π]
//
// Outward unit normal: n = cos(v)*cos(u)*uRef + cos(v)*sin(u)*vRef + sin(v)*A
//
// Principal curvatures:  κ₁ = 1/r,  κ₂ = cos(v) / (R + r*cos(v))
// Gaussian curvature: K = cos(v) / (r*(R + r*cos(v)))
// Mean curvature:     H = (R + 2r*cos(v)) / (2r*(R + r*cos(v)))

#include "gk/surface/ISurface.h"
#include <cmath>
#include <utility>

namespace gk {

class Torus : public ISurface
{
public:
    static constexpr double kPi = 3.14159265358979323846;

    /// @param center      Centre of the torus.
    /// @param axis        Unit axis (normal to the equatorial plane).
    /// @param majorRadius Distance from the torus centre to the tube centre (R).
    /// @param minorRadius Radius of the tube (r).  Must satisfy r < R.
    Torus(Vec3 center, Vec3 axis, double majorRadius, double minorRadius) noexcept
        : center_(center)
        , axis_(axis.normalized())
        , R_(majorRadius)
        , r_(minorRadius)
    {
        buildFrame(axis_, uRef_, vRef_);
    }

    /// Constructor with explicit x-reference direction.
    Torus(Vec3 center, Vec3 axis, double majorRadius, double minorRadius,
          Vec3 xRef) noexcept
        : center_(center)
        , axis_(axis.normalized())
        , R_(majorRadius)
        , r_(minorRadius)
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
        double cv = std::cos(v), sv = std::sin(v);

        Vec3 radU   =  uRef_ * cu  + vRef_ * su;    // unit major radial dir
        Vec3 dtangU = -uRef_ * su  + vRef_ * cu;    // ∂radU/∂u

        double rho = R_ + r_ * cv;                  // distance from axis

        SurfacePoint sp;
        sp.p  = center_ + radU * rho + axis_ * (r_ * sv);

        // ∂P/∂u = rho * ∂radU/∂u
        sp.du = dtangU * rho;
        // ∂P/∂v = -r*sin(v)*radU + r*cos(v)*axis
        sp.dv = radU * (-r_ * sv) + axis_ * (r_ * cv);

        // ∂²P/∂u²  = -rho * radU
        sp.duu = radU * (-rho);
        // ∂²P/∂u∂v = -(r*sin(v)) * dtangU   (since ∂rho/∂v = -r*sin(v))
        sp.duv = dtangU * (-r_ * sv);
        // ∂²P/∂v²  = -r*cos(v)*radU - r*sin(v)*axis
        sp.dvv = radU * (-r_ * cv) + axis_ * (-r_ * sv);

        return sp;
    }

    SurfaceDomain domain() const override
    {
        return { Interval{0.0, 2.0*kPi}, Interval{0.0, 2.0*kPi} };
    }

    Vec3 normalAt(double u, double v) const override
    {
        double cu = std::cos(u), su = std::sin(u);
        double cv = std::cos(v), sv = std::sin(v);
        Vec3 radU = uRef_ * cu + vRef_ * su;
        // Outward unit normal: cos(v)*radU + sin(v)*axis
        return radU * cv + axis_ * sv;
    }

    void isClosed(bool& cu, bool& cv) const override { cu = true; cv = true; }

    // ── Analytic utilities ────────────────────────────────────────────────────

    std::pair<double,double> inverseEvaluate(const Vec3& pt) const noexcept
    {
        Vec3 d = pt - center_;
        // Major angle
        double pu = d.dot(uRef_), pv = d.dot(vRef_);
        double u  = std::atan2(pv, pu);
        if (u < 0.0) u += 2.0 * kPi;

        // Vector from tube circle centre to point
        Vec3 tubeCenter = center_ + (uRef_ * std::cos(u) + vRef_ * std::sin(u)) * R_;
        Vec3 td = pt - tubeCenter;

        Vec3 radU  = uRef_ * std::cos(u) + vRef_ * std::sin(u);
        double radComp = td.dot(radU);
        double axComp  = td.dot(axis_);
        double v  = std::atan2(axComp, radComp);
        if (v < 0.0) v += 2.0 * kPi;
        return {u, v};
    }

    double gaussianCurvature(double /*u*/, double v) const noexcept
    {
        double cv = std::cos(v);
        double denom = r_ * (R_ + r_ * cv);
        return (denom != 0.0) ? cv / denom : 0.0;
    }

    double meanCurvature(double /*u*/, double v) const noexcept
    {
        double cv = std::cos(v);
        double numer = R_ + 2.0 * r_ * cv;
        double denom = 2.0 * r_ * (R_ + r_ * cv);
        return (denom != 0.0) ? numer / denom : 0.0;
    }

    // ── Accessors ─────────────────────────────────────────────────────────────
    const Vec3& center() const noexcept { return center_; }
    const Vec3& axis()   const noexcept { return axis_;   }
    const Vec3& uRef()   const noexcept { return uRef_;   }
    double       majorRadius() const noexcept { return R_; }
    double       minorRadius() const noexcept { return r_; }

private:
    Vec3   center_, axis_, uRef_, vRef_;
    double R_, r_;

    static void buildFrame(const Vec3& axis, Vec3& u, Vec3& v) noexcept
    {
        Vec3 ref = (std::abs(axis.dot(Vec3::unitX())) < 0.9)
                   ? Vec3::unitX() : Vec3::unitY();
        u = (ref - axis * ref.dot(axis)).normalized();
        v = axis.cross(u);
    }
};

} // namespace gk
