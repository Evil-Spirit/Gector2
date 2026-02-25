#pragma once
// Chapter 5 — Ruled surface between two 3D curves.
//
// Parameterization:
//   P(u,v) = (1-v)*c0(u) + v*c1(u)
//   u ∈ c0->domain()  (assumed same as c1->domain())
//   v ∈ [0, 1]

#include "gk/surface/ISurface.h"
#include "gk/curve/ICurve.h"
#include <memory>

namespace gk {

class RuledSurface : public ISurface
{
public:
    RuledSurface(std::shared_ptr<ICurve3> c0, std::shared_ptr<ICurve3> c1) noexcept
        : c0_(std::move(c0)), c1_(std::move(c1)) {}

    // ── ISurface ─────────────────────────────────────────────────────────────
    SurfacePoint evaluate(double u, double v) const override
    {
        auto p0 = c0_->evaluate(u);
        auto p1 = c1_->evaluate(u);

        double w0 = 1.0 - v;
        double w1 = v;

        SurfacePoint sp;
        // P(u,v) = (1-v)*c0(u) + v*c1(u)
        sp.p  = p0.p * w0 + p1.p * w1;
        // ∂P/∂u = (1-v)*c0'(u) + v*c1'(u)
        sp.du = p0.d1 * w0 + p1.d1 * w1;
        // ∂P/∂v = c1(u) - c0(u)
        sp.dv = p1.p - p0.p;

        // ∂²P/∂u² = (1-v)*c0''(u) + v*c1''(u)
        sp.duu = p0.d2 * w0 + p1.d2 * w1;
        // ∂²P/∂u∂v = c1'(u) - c0'(u)
        sp.duv = p1.d1 - p0.d1;
        // ∂²P/∂v² = 0
        sp.dvv = Vec3::zero();
        return sp;
    }

    SurfaceDomain domain() const override
    {
        return { c0_->domain(), Interval{0.0, 1.0} };
    }

    Vec3 normalAt(double u, double v) const override
    {
        auto sp = evaluate(u, v);
        Vec3 n  = sp.du.cross(sp.dv);
        double len = n.norm();
        return (len > 1e-14) ? n * (1.0 / len) : Vec3::zero();
    }

    void isClosed(bool& cu, bool& cv) const override
    {
        cu = c0_->isClosed();
        cv = false;
    }

    // ── Accessors ─────────────────────────────────────────────────────────────
    const std::shared_ptr<ICurve3>& curve0() const noexcept { return c0_; }
    const std::shared_ptr<ICurve3>& curve1() const noexcept { return c1_; }

private:
    std::shared_ptr<ICurve3> c0_, c1_;
};

} // namespace gk
