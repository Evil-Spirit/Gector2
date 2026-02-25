#pragma once
// Iteration 3.3 — Tensor-product NURBS (Non-Uniform Rational B-Spline) surface.
//
// Extends BSplineSurface with per-control-point weights.  Evaluation uses
// the standard rational formulas (A-weights) for point, 1st, and 2nd
// order derivatives ("The NURBS Book", Algorithm A4.4).

#include "gk/surface/ISurface.h"
#include "gk/surface/BSplineSurface.h"
#include <stdexcept>
#include <vector>

namespace gk {

class NURBSSurface : public ISurface
{
public:
    using CtrlGrid = std::vector<std::vector<Vec3>>;
    using WtGrid   = std::vector<std::vector<double>>;

    // ── Construction ─────────────────────────────────────────────────────────

    NURBSSurface(int degreeU, int degreeV,
                 std::vector<double> knotsU,
                 std::vector<double> knotsV,
                 CtrlGrid            ctrlPts,
                 WtGrid              weights)
        : degU_(degreeU)
        , degV_(degreeV)
        , knotsU_(std::move(knotsU))
        , knotsV_(std::move(knotsV))
        , ctrlPts_(std::move(ctrlPts))
        , weights_(std::move(weights))
    {
        validate();
    }

    /// Construct a NURBS surface with uniform unit weights (≡ B-spline surface).
    NURBSSurface(int degreeU, int degreeV,
                 std::vector<double> knotsU,
                 std::vector<double> knotsV,
                 CtrlGrid            ctrlPts)
        : NURBSSurface(degreeU, degreeV,
                       std::move(knotsU), std::move(knotsV),
                       ctrlPts,
                       WtGrid(ctrlPts.size(),
                              std::vector<double>(
                                  ctrlPts.empty() ? 0 : ctrlPts[0].size(), 1.0)))
    {}

    // ── ISurface ─────────────────────────────────────────────────────────────

    SurfacePoint evaluate(double u, double v) const override
    {
        int nu = numU(), nv = numV();
        int spanU = BSplineSurface::findSpan(nu-1, degU_, u, knotsU_);
        int spanV = BSplineSurface::findSpan(nv-1, degV_, v, knotsV_);

        auto Nu   = BSplineSurface::basisFuns(spanU, u, degU_,   knotsU_);
        auto NudR = (degU_ >= 1) ? BSplineSurface::basisFuns(spanU, u, degU_-1, knotsU_)
                                 : std::vector<double>(degU_, 0.0);
        auto NuddR= (degU_ >= 2) ? BSplineSurface::basisFuns(spanU, u, degU_-2, knotsU_)
                                 : std::vector<double>((degU_ >= 1 ? degU_-1 : 1), 0.0);
        auto Nv   = BSplineSurface::basisFuns(spanV, v, degV_,   knotsV_);
        auto NvdR = (degV_ >= 1) ? BSplineSurface::basisFuns(spanV, v, degV_-1, knotsV_)
                                 : std::vector<double>(degV_, 0.0);
        auto NvddR= (degV_ >= 2) ? BSplineSurface::basisFuns(spanV, v, degV_-2, knotsV_)
                                 : std::vector<double>((degV_ >= 1 ? degV_-1 : 1), 0.0);

        auto dNu  = BSplineSurface::deriv1(spanU, degU_, knotsU_, NudR);
        auto ddNu = BSplineSurface::deriv2(spanU, degU_, knotsU_, NudR, NuddR);
        auto dNv  = BSplineSurface::deriv1(spanV, degV_, knotsV_, NvdR);
        auto ddNv = BSplineSurface::deriv2(spanV, degV_, knotsV_, NvdR, NvddR);

        // Accumulate weighted sums in homogeneous space.
        Vec3   Aw,  Aw_u,  Aw_v,  Aw_uu,  Aw_uv,  Aw_vv;
        double W = 0.0, Wu = 0.0, Wv = 0.0, Wuu = 0.0, Wuv = 0.0, Wvv = 0.0;

        for (int ki = 0; ki <= degU_; ++ki)
        {
            int ci = spanU - degU_ + ki;
            for (int kj = 0; kj <= degV_; ++kj)
            {
                int cj = spanV - degV_ + kj;
                double wij = weights_[ci][cj];
                const Vec3& Pij = ctrlPts_[ci][cj];

                double Ri   = Nu[ki]   * Nv[kj];
                double Ri_u = dNu[ki]  * Nv[kj];
                double Ri_v = Nu[ki]   * dNv[kj];
                double Ri_uu= ddNu[ki] * Nv[kj];
                double Ri_uv= dNu[ki]  * dNv[kj];
                double Ri_vv= Nu[ki]   * ddNv[kj];

                Aw    = Aw    + Pij * (wij * Ri);
                Aw_u  = Aw_u  + Pij * (wij * Ri_u);
                Aw_v  = Aw_v  + Pij * (wij * Ri_v);
                Aw_uu = Aw_uu + Pij * (wij * Ri_uu);
                Aw_uv = Aw_uv + Pij * (wij * Ri_uv);
                Aw_vv = Aw_vv + Pij * (wij * Ri_vv);

                W   += wij * Ri;
                Wu  += wij * Ri_u;
                Wv  += wij * Ri_v;
                Wuu += wij * Ri_uu;
                Wuv += wij * Ri_uv;
                Wvv += wij * Ri_vv;
            }
        }

        // Rational derivatives ("The NURBS Book" Algorithm A4.4 extended).
        SurfacePoint sp;
        if (std::abs(W) < 1e-15)
        {
            sp.p = sp.du = sp.dv = sp.duu = sp.duv = sp.dvv = Vec3::zero();
            return sp;
        }

        sp.p  = Aw  * (1.0 / W);

        sp.du = (Aw_u  - sp.p * Wu)  * (1.0 / W);
        sp.dv = (Aw_v  - sp.p * Wv)  * (1.0 / W);

        sp.duu = (Aw_uu - sp.du * (2.0*Wu) - sp.p * Wuu) * (1.0/W);
        sp.duv = (Aw_uv - sp.du * Wv - sp.dv * Wu - sp.p * Wuv) * (1.0/W);
        sp.dvv = (Aw_vv - sp.dv * (2.0*Wv) - sp.p * Wvv) * (1.0/W);

        return sp;
    }

    SurfaceDomain domain() const override
    {
        return { Interval{knotsU_.front(), knotsU_.back()},
                 Interval{knotsV_.front(), knotsV_.back()} };
    }

    void isClosed(bool& cu, bool& cv) const override { cu = false; cv = false; }

    // ── Accessors ─────────────────────────────────────────────────────────────
    int degreeU() const noexcept { return degU_; }
    int degreeV() const noexcept { return degV_; }
    const std::vector<double>& knotsU()       const noexcept { return knotsU_; }
    const std::vector<double>& knotsV()       const noexcept { return knotsV_; }
    const CtrlGrid&            controlPoints() const noexcept { return ctrlPts_; }
    const WtGrid&              weights()       const noexcept { return weights_; }
    int numU() const noexcept { return (int)ctrlPts_.size(); }
    int numV() const noexcept
    {
        return ctrlPts_.empty() ? 0 : (int)ctrlPts_[0].size();
    }

private:
    int                 degU_, degV_;
    std::vector<double> knotsU_, knotsV_;
    CtrlGrid            ctrlPts_;
    WtGrid              weights_;

    void validate() const
    {
        int nu = numU(), nv = numV();
        if ((int)knotsU_.size() != nu + degU_ + 1)
            throw std::invalid_argument("NURBSSurface: bad knotsU size");
        if ((int)knotsV_.size() != nv + degV_ + 1)
            throw std::invalid_argument("NURBSSurface: bad knotsV size");
        if ((int)weights_.size() != nu)
            throw std::invalid_argument("NURBSSurface: bad weights row count");
        for (auto& row : weights_)
            if ((int)row.size() != nv)
                throw std::invalid_argument("NURBSSurface: bad weights column count");
    }
};

} // namespace gk
