#pragma once
// Iteration 3.1 — Surface Abstract Interface.
//
// Every concrete surface implements ISurface.  The evaluate() method returns
// a SurfacePoint containing the position and all partial derivatives up to 2nd
// order needed for curvature computation and Newton-based closest-point.

#include "gk/math/Vec3.h"
#include "gk/math/Interval.h"

namespace gk {

// ── SurfacePoint ──────────────────────────────────────────────────────────────

/// Evaluation result at a single (u,v) parameter.
struct SurfacePoint
{
    Vec3 p;   ///< Position  S(u,v)
    Vec3 du;  ///< ∂S/∂u
    Vec3 dv;  ///< ∂S/∂v
    Vec3 duu; ///< ∂²S/∂u²
    Vec3 duv; ///< ∂²S/∂u∂v
    Vec3 dvv; ///< ∂²S/∂v²
};

// ── SurfaceDomain ─────────────────────────────────────────────────────────────

/// Axis-aligned parameter domain [u_lo, u_hi] × [v_lo, v_hi].
struct SurfaceDomain
{
    Interval u;
    Interval v;
};

// ── ISurface ──────────────────────────────────────────────────────────────────

/// Abstract surface interface.
class ISurface
{
public:
    virtual ~ISurface() = default;

    /// Evaluate position and partial derivatives up to 2nd order at (u,v).
    virtual SurfacePoint evaluate(double u, double v) const = 0;

    /// Axis-aligned parameter domain.
    virtual SurfaceDomain domain() const = 0;

    /// Outward unit normal at (u,v).
    /// Default: du × dv, normalised.  Overridden by analytic surfaces for
    /// correctness at degenerate parameters (e.g. sphere poles).
    virtual Vec3 normalAt(double u, double v) const
    {
        auto sp = evaluate(u, v);
        return sp.du.cross(sp.dv).normalized();
    }

    /// Whether the surface is closed (periodic) in u and/or v.
    virtual void isClosed(bool& closedU, bool& closedV) const = 0;
};

} // namespace gk
