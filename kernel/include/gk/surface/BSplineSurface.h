#pragma once
// Iteration 3.3 — Tensor-product B-Spline surface.
//
// Storage: controlPoints_[i][j]  where i ∈ [0, numU-1], j ∈ [0, numV-1].
// Knot vectors: knotsU_ has size numU + degreeU + 1,
//               knotsV_ has size numV + degreeV + 1.
//
// Evaluation uses tensor-product de Boor basis functions with derivatives up
// to 2nd order (needed by ISurface::evaluate).
//
// Knot insertion (Boehm's algorithm) and degree elevation are provided to
// satisfy the requirements of Iteration 3.3.

#include "gk/surface/ISurface.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace gk {

class BSplineSurface : public ISurface
{
public:
    using CtrlGrid = std::vector<std::vector<Vec3>>;

    // ── Construction ─────────────────────────────────────────────────────────

    BSplineSurface(int degreeU, int degreeV,
                   std::vector<double> knotsU,
                   std::vector<double> knotsV,
                   CtrlGrid            ctrlPts)
        : degU_(degreeU)
        , degV_(degreeV)
        , knotsU_(std::move(knotsU))
        , knotsV_(std::move(knotsV))
        , ctrlPts_(std::move(ctrlPts))
    {
        validate();
    }

    /// Build a uniform (clamped) knot vector for n control points at degree p.
    static std::vector<double> uniformKnots(int n, int p)
    {
        // Clamped: first and last p+1 knots are 0 and 1 respectively.
        int m = n + p + 1;
        std::vector<double> U(m);
        for (int i = 0; i <= p;     ++i) U[i] = 0.0;
        for (int i = p+1; i < n;    ++i) U[i] = double(i-p) / double(n-p);
        for (int i = n;   i < m;    ++i) U[i] = 1.0;
        return U;
    }

    // ── ISurface ─────────────────────────────────────────────────────────────

    SurfacePoint evaluate(double u, double v) const override
    {
        int nu = numU(), nv = numV();
        int spanU = findSpan(nu-1, degU_, u, knotsU_);
        int spanV = findSpan(nv-1, degV_, v, knotsV_);

        // Basis functions and derivatives up to order 2 in each direction.
        auto Nu  = basisFuns(spanU, u, degU_,   knotsU_);
        auto Nud = (degU_ >= 1) ? basisFuns(spanU, u, degU_-1, knotsU_)
                                : std::vector<double>(degU_, 0.0);
        auto Nudd= (degU_ >= 2) ? basisFuns(spanU, u, degU_-2, knotsU_)
                                : std::vector<double>((degU_ >= 1 ? degU_-1 : 1), 0.0);
        auto Nv  = basisFuns(spanV, v, degV_,   knotsV_);
        auto Nvd = (degV_ >= 1) ? basisFuns(spanV, v, degV_-1, knotsV_)
                                : std::vector<double>(degV_, 0.0);
        auto Nvdd= (degV_ >= 2) ? basisFuns(spanV, v, degV_-2, knotsV_)
                                : std::vector<double>((degV_ >= 1 ? degV_-1 : 1), 0.0);

        // Derivative coefficients using the B-spline derivative recurrence.
        // N'_{i,p}(u) = p * [N_{i,p-1}/(U[i+p]-U[i]) - N_{i+1,p-1}/(U[i+p+1]-U[i+1])]
        // The p nonzero N_{·,p-1} at span spanU are indexed [0..p-1] in Nud.
        // They correspond to indices spanU-p+1 .. spanU globally.

        auto dNu  = deriv1(spanU, degU_, knotsU_, Nud);
        auto ddNu = deriv2(spanU, degU_, knotsU_, Nud, Nudd);
        auto dNv  = deriv1(spanV, degV_, knotsV_, Nvd);
        auto ddNv = deriv2(spanV, degV_, knotsV_, Nvd, Nvdd);

        SurfacePoint sp;
        for (int ki = 0; ki <= degU_; ++ki)
        {
            int ci = spanU - degU_ + ki;
            Vec3 tmpP, tmpDu, tmpDv, tmpDuu, tmpDuv, tmpDvv;
            for (int kj = 0; kj <= degV_; ++kj)
            {
                int cj = spanV - degV_ + kj;
                const Vec3& P = ctrlPts_[ci][cj];
                tmpP   = tmpP   + P * (Nu[ki]  * Nv[kj]);
                tmpDu  = tmpDu  + P * (dNu[ki] * Nv[kj]);
                tmpDv  = tmpDv  + P * (Nu[ki]  * dNv[kj]);
                tmpDuu = tmpDuu + P * (ddNu[ki]* Nv[kj]);
                tmpDuv = tmpDuv + P * (dNu[ki] * dNv[kj]);
                tmpDvv = tmpDvv + P * (Nu[ki]  * ddNv[kj]);
            }
            sp.p   = sp.p   + tmpP;
            sp.du  = sp.du  + tmpDu;
            sp.dv  = sp.dv  + tmpDv;
            sp.duu = sp.duu + tmpDuu;
            sp.duv = sp.duv + tmpDuv;
            sp.dvv = sp.dvv + tmpDvv;
        }
        return sp;
    }

    SurfaceDomain domain() const override
    {
        auto& U = knotsU_; auto& V = knotsV_;
        return { Interval{U.front(), U.back()},
                 Interval{V.front(), V.back()} };
    }

    void isClosed(bool& cu, bool& cv) const override { cu = false; cv = false; }

    // ── Knot insertion (Boehm's algorithm) ───────────────────────────────────

    /// Return a new surface with knot t inserted into the U knot vector once.
    BSplineSurface insertKnotU(double t) const
    {
        int nu   = numU(), p = degU_;
        int span = findSpan(nu-1, p, t, knotsU_);

        // New knot vector
        std::vector<double> newU = knotsU_;
        newU.insert(newU.begin() + span + 1, t);

        // New control point grid: one extra row in U direction
        CtrlGrid newPts(nu + 1, std::vector<Vec3>(numV()));
        for (int j = 0; j < numV(); ++j)
        {
            // Extract column j as a 1-D curve and insert knot there
            std::vector<Vec3> col(nu);
            for (int i = 0; i < nu; ++i) col[i] = ctrlPts_[i][j];
            auto newCol = insertKnot1D(col, knotsU_, p, span, t);
            for (int i = 0; i <= nu; ++i) newPts[i][j] = newCol[i];
        }
        return BSplineSurface{degU_, degV_, std::move(newU), knotsV_, std::move(newPts)};
    }

    /// Return a new surface with knot t inserted into the V knot vector once.
    BSplineSurface insertKnotV(double t) const
    {
        int nv   = numV(), q = degV_;
        int span = findSpan(nv-1, q, t, knotsV_);

        std::vector<double> newV = knotsV_;
        newV.insert(newV.begin() + span + 1, t);

        CtrlGrid newPts(numU(), std::vector<Vec3>(nv + 1));
        for (int i = 0; i < numU(); ++i)
        {
            auto newRow = insertKnot1D(ctrlPts_[i], knotsV_, q, span, t);
            newPts[i] = std::move(newRow);
        }
        return BSplineSurface{degU_, degV_, knotsU_, std::move(newV), std::move(newPts)};
    }

    // ── Degree elevation ──────────────────────────────────────────────────────

    /// Return a new surface with the U degree elevated by one.
    BSplineSurface elevateU() const
    {
        // Elevate each row (fixed j, vary i) independently.
        int nv = numV();
        std::vector<Vec3> firstRow(nv);
        auto [newKU, newRow0] = elevate1D(column(0), knotsU_, degU_);
        int newNU = (int)newRow0.size();

        CtrlGrid newPts(newNU, std::vector<Vec3>(nv));
        for (int j = 0; j < nv; ++j)
        {
            auto [kU2, row] = elevate1D(column(j), knotsU_, degU_);
            for (int i = 0; i < newNU; ++i) newPts[i][j] = row[i];
        }
        return BSplineSurface{degU_+1, degV_, std::move(newKU), knotsV_, std::move(newPts)};
    }

    /// Return a new surface with the V degree elevated by one.
    BSplineSurface elevateV() const
    {
        int nu = numU();
        auto [newKV, firstElevRow] = elevate1D(ctrlPts_[0], knotsV_, degV_);
        int newNV = (int)firstElevRow.size();

        CtrlGrid newPts(nu, std::vector<Vec3>(newNV));
        for (int i = 0; i < nu; ++i)
        {
            auto [kV2, row] = elevate1D(ctrlPts_[i], knotsV_, degV_);
            newPts[i] = std::move(row);
        }
        return BSplineSurface{degU_, degV_+1, knotsU_, std::move(newKV), std::move(newPts)};
    }

    // ── Accessors ─────────────────────────────────────────────────────────────
    int degreeU() const noexcept { return degU_; }
    int degreeV() const noexcept { return degV_; }
    const std::vector<double>& knotsU()       const noexcept { return knotsU_; }
    const std::vector<double>& knotsV()       const noexcept { return knotsV_; }
    const CtrlGrid&            controlPoints() const noexcept { return ctrlPts_; }
    int numU() const noexcept { return (int)ctrlPts_.size(); }
    int numV() const noexcept
    {
        return ctrlPts_.empty() ? 0 : (int)ctrlPts_[0].size();
    }

    // ── Static B-spline helpers (public so NURBSSurface can reuse them) ──────

    /// Find the knot span index i such that U[i] <= u < U[i+1].
    static int findSpan(int n, int p, double u, const std::vector<double>& U)
    {
        if (u >= U[n+1]) return n;         // special case: at the upper end
        if (u <= U[p])   return p;
        int lo = p, hi = n + 1, mid = (lo + hi) / 2;
        while (u < U[mid] || u >= U[mid+1])
        {
            if (u < U[mid]) hi = mid; else lo = mid;
            mid = (lo + hi) / 2;
        }
        return mid;
    }

    /// Compute the (deg+1) nonzero basis functions N_{i-deg,deg}..N_{i,deg} at u.
    /// Returns a vector of size deg+1.
    static std::vector<double> basisFuns(int i, double u, int deg,
                                          const std::vector<double>& U)
    {
        if (deg < 0) return {};
        std::vector<double> N(deg+1, 0.0);
        std::vector<double> left(deg+1, 0.0), right(deg+1, 0.0);
        N[0] = 1.0;
        for (int j = 1; j <= deg; ++j)
        {
            left[j]  = u - U[i+1-j];
            right[j] = U[i+j] - u;
            double saved = 0.0;
            for (int r = 0; r < j; ++r)
            {
                double denom = right[r+1] + left[j-r];
                double temp  = (std::abs(denom) > 0.0) ? N[r] / denom : 0.0;
                N[r] = saved + right[r+1] * temp;
                saved = left[j-r] * temp;
            }
            N[j] = saved;
        }
        return N;
    }

    /// Compute 1st-order derivatives of N_{i-p,p}..N_{i,p}
    /// from the degree-(p-1) basis functions Nd (size p).
    static std::vector<double> deriv1(int i, int p,
                                       const std::vector<double>& U,
                                       const std::vector<double>& Nd)
    {
        std::vector<double> dN(p+1, 0.0);
        if (p == 0) return dN;
        {
            double denom = U[i+1] - U[i-p+1];
            dN[0] = (std::abs(denom) > 0.0) ? -double(p) * Nd[0] / denom : 0.0;
        }
        for (int k = 1; k < p; ++k)
        {
            double d1 = U[i+k]   - U[i-p+k];
            double d2 = U[i+k+1] - U[i-p+k+1];
            double t1 = (std::abs(d1) > 0.0) ? Nd[k-1] / d1 : 0.0;
            double t2 = (std::abs(d2) > 0.0) ? Nd[k]   / d2 : 0.0;
            dN[k] = double(p) * (t1 - t2);
        }
        {
            double denom = U[i+p] - U[i];
            dN[p] = (std::abs(denom) > 0.0) ? double(p) * Nd[p-1] / denom : 0.0;
        }
        return dN;
    }

    /// Compute 2nd-order derivatives from degree-(p-1) basis Nd
    /// and degree-(p-2) basis Ndd.
    static std::vector<double> deriv2(int i, int p,
                                       const std::vector<double>& U,
                                       const std::vector<double>& /*Nd*/,
                                       const std::vector<double>& Ndd)
    {
        std::vector<double> ddN(p+1, 0.0);
        if (p < 2) return ddN;
        auto dNd = deriv1(i, p-1, U, Ndd);
        {
            double denom = U[i+1] - U[i-p+1];
            ddN[0] = (std::abs(denom) > 0.0) ? -double(p) * dNd[0] / denom : 0.0;
        }
        for (int k = 1; k < p; ++k)
        {
            double d1 = U[i+k]   - U[i-p+k];
            double d2 = U[i+k+1] - U[i-p+k+1];
            double t1 = (std::abs(d1) > 0.0) ? dNd[k-1] / d1 : 0.0;
            double t2 = (std::abs(d2) > 0.0) ? dNd[k]   / d2 : 0.0;
            ddN[k] = double(p) * (t1 - t2);
        }
        {
            double denom = U[i+p] - U[i];
            ddN[p] = (std::abs(denom) > 0.0) ? double(p) * dNd[p-1] / denom : 0.0;
        }
        return ddN;
    }

private:
    int                 degU_, degV_;
    std::vector<double> knotsU_, knotsV_;
    CtrlGrid            ctrlPts_;

    void validate() const
    {
        int nu = numU(), nv = numV();
        if ((int)knotsU_.size() != nu + degU_ + 1)
            throw std::invalid_argument("BSplineSurface: bad knotsU size");
        if ((int)knotsV_.size() != nv + degV_ + 1)
            throw std::invalid_argument("BSplineSurface: bad knotsV size");
    }

    // ── Knot insertion helper (1-D curve) ─────────────────────────────────────

    static std::vector<Vec3> insertKnot1D(const std::vector<Vec3>& P,
                                           const std::vector<double>& U,
                                           int p, int k, double t)
    {
        int n = (int)P.size() - 1;
        std::vector<Vec3> Q(n + 2);
        // Points before affected region
        for (int i = 0; i <= k-p; ++i) Q[i] = P[i];
        // Affected region
        for (int i = k-p+1; i <= k; ++i)
        {
            double denom = U[i+p] - U[i];
            double alpha = (std::abs(denom) > 0.0) ? (t - U[i]) / denom : 0.0;
            Q[i] = P[i-1] * (1.0 - alpha) + P[i] * alpha;
        }
        // Points after affected region
        for (int i = k+1; i <= n+1; ++i) Q[i] = P[i-1];
        return Q;
    }

    // ── Degree elevation helper (1-D curve, Bézier-based) ────────────────────
    /// Elevate a single 1-D B-spline curve by one degree.
    /// Uses: extract Bézier segments → elevate → merge.
    static std::pair<std::vector<double>, std::vector<Vec3>>
    elevate1D(const std::vector<Vec3>& P, const std::vector<double>& U, int p)
    {
        std::vector<double> Ucur = U;
        std::vector<Vec3>   Pcur = P;

        // Collect unique interior knot values and their multiplicities in U.
        // The "clamped" knot vector has U[0..p] = 0 and U[m-p..m] = end.
        std::vector<double> interiorKnots;
        {
            int m = (int)Ucur.size() - 1;  // m = n+p
            for (int i = p+1; i <= m-p-1; ++i)
            {
                if (interiorKnots.empty() || Ucur[i] != interiorKnots.back())
                    interiorKnots.push_back(Ucur[i]);
            }
        }
        // Insert each interior knot until it has multiplicity p
        for (double t : interiorKnots)
        {
            // Count current multiplicity
            int mult = 0;
            for (double k : Ucur) if (std::abs(k - t) < 1e-14) ++mult;
            int needed = p - mult;
            for (int r = 0; r < needed; ++r)
            {
                int nn = (int)Pcur.size() - 1;
                int span = findSpan(nn, p, t, Ucur);
                Pcur = insertKnot1D(Pcur, Ucur, p, span, t);
                Ucur.insert(Ucur.begin() + span + 1, t);
            }
        }

        // Now Ucur/Pcur is decomposed into Bézier segments of degree p.
        // Each segment uses p+1 control points; they share endpoints.
        int numSeg = (int)interiorKnots.size() + 1;

        // --- Step 2: elevate each Bézier segment from degree p to p+1
        // Elevated segment has p+2 control points.
        //   c[0] = b[0]
        //   c[i] = ((p+1-i)*b[i] + i*b[i-1]) / (p+1)   for i=1..p
        //   c[p+1] = b[p]
        std::vector<std::vector<Vec3>> elevSegs(numSeg, std::vector<Vec3>(p+2));
        for (int s = 0; s < numSeg; ++s)
        {
            const Vec3* b = Pcur.data() + s * p;   // segment shares endpoints
            elevSegs[s][0] = b[0];
            for (int i = 1; i <= p; ++i)
                elevSegs[s][i] = (b[i] * double(p+1-i) + b[i-1] * double(i))
                                 * (1.0 / double(p+1));
            elevSegs[s][p+1] = b[p];
        }

        // --- Step 3: assemble new knot vector and control points
        // New degree = p+1. Each segment contributes p+2 points but adjacent
        // segments share 1 endpoint (they are C^0 from the Bézier elevation).
        // New knot vector: each interior knot has multiplicity p+1 (was p).
        int newP = p + 1;
        std::vector<double> newU;
        newU.reserve(numSeg * (newP+1) + newP + 1);
        for (int i = 0; i <= newP; ++i) newU.push_back(Ucur.front());
        for (int s = 0; s < numSeg - 1; ++s)
        {
            double t = interiorKnots[s];
            // multiplicity p was used during Bezier extraction, now p+1 for elevated
            for (int r = 0; r <= newP; ++r) newU.push_back(t);
        }
        for (int i = 0; i <= newP; ++i) newU.push_back(Ucur.back());

        std::vector<Vec3> newP_pts;
        newP_pts.reserve(numSeg * (p+1) + 1);
        for (int s = 0; s < numSeg; ++s)
            for (int i = 0; i <= p; ++i)   // skip last point of each segment except final
                newP_pts.push_back(elevSegs[s][i]);
        newP_pts.push_back(elevSegs[numSeg-1][p+1]);

        return {std::move(newU), std::move(newP_pts)};
    }

    /// Extract the j-th column from ctrlPts_ as a row vector (for elevateU).
    std::vector<Vec3> column(int j) const
    {
        std::vector<Vec3> col(numU());
        for (int i = 0; i < numU(); ++i) col[i] = ctrlPts_[i][j];
        return col;
    }
};

} // namespace gk
