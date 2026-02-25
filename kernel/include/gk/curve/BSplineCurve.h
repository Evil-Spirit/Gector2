#pragma once
#include "gk/curve/ICurve.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace gk {

namespace detail {

inline int bsplFindSpan(int n, int p, double u, const std::vector<double>& U)
{
    if (u >= U[n+1]) return n;
    if (u <= U[p])   return p;
    int lo = p, hi = n + 1, mid = (lo + hi) / 2;
    while (u < U[mid] || u >= U[mid+1]) {
        if (u < U[mid]) hi = mid; else lo = mid;
        mid = (lo + hi) / 2;
    }
    return mid;
}

inline std::vector<double> bsplBasis(int i, double u, int deg,
                                      const std::vector<double>& U)
{
    if (deg < 0) return {};
    std::vector<double> N(deg+1, 0.0);
    std::vector<double> left(deg+1, 0.0), right(deg+1, 0.0);
    N[0] = 1.0;
    for (int j = 1; j <= deg; ++j) {
        left[j]  = u - U[i+1-j];
        right[j] = U[i+j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            double denom = right[r+1] + left[j-r];
            double temp  = (denom > 0.0 || denom < 0.0) ? N[r] / denom : 0.0;
            N[r] = saved + right[r+1] * temp;
            saved = left[j-r] * temp;
        }
        N[j] = saved;
    }
    return N;
}

inline std::vector<double> bsplDeriv1(int i, int p,
                                        const std::vector<double>& U,
                                        const std::vector<double>& Nd)
{
    std::vector<double> dN(p+1, 0.0);
    if (p == 0) return dN;
    {
        double denom = U[i+1] - U[i-p+1];
        dN[0] = (denom > 0.0 || denom < 0.0) ? -double(p) * Nd[0] / denom : 0.0;
    }
    for (int k = 1; k < p; ++k) {
        double d1 = U[i+k]   - U[i-p+k];
        double d2 = U[i+k+1] - U[i-p+k+1];
        double t1 = (d1 > 0.0 || d1 < 0.0) ? Nd[k-1] / d1 : 0.0;
        double t2 = (d2 > 0.0 || d2 < 0.0) ? Nd[k]   / d2 : 0.0;
        dN[k] = double(p) * (t1 - t2);
    }
    {
        double denom = U[i+p] - U[i];
        dN[p] = (denom > 0.0 || denom < 0.0) ? double(p) * Nd[p-1] / denom : 0.0;
    }
    return dN;
}

inline std::vector<double> bsplDeriv2(int i, int p,
                                        const std::vector<double>& U,
                                        const std::vector<double>& Ndd)
{
    std::vector<double> ddN(p+1, 0.0);
    if (p < 2) return ddN;
    auto dNd = bsplDeriv1(i, p-1, U, Ndd);
    {
        double denom = U[i+1] - U[i-p+1];
        ddN[0] = (denom > 0.0 || denom < 0.0) ? -double(p) * dNd[0] / denom : 0.0;
    }
    for (int k = 1; k < p; ++k) {
        double d1 = U[i+k]   - U[i-p+k];
        double d2 = U[i+k+1] - U[i-p+k+1];
        double t1 = (d1 > 0.0 || d1 < 0.0) ? dNd[k-1] / d1 : 0.0;
        double t2 = (d2 > 0.0 || d2 < 0.0) ? dNd[k]   / d2 : 0.0;
        ddN[k] = double(p) * (t1 - t2);
    }
    {
        double denom = U[i+p] - U[i];
        ddN[p] = (denom > 0.0 || denom < 0.0) ? double(p) * dNd[p-1] / denom : 0.0;
    }
    return ddN;
}

} // namespace detail

// ── BSplineCurve3 ─────────────────────────────────────────────────────────────

class BSplineCurve3 : public ICurve3 {
public:
    BSplineCurve3(int degree, std::vector<double> knots,
                  std::vector<Vec3> controlPoints)
        : deg_(degree)
        , knots_(std::move(knots))
        , ctrl_(std::move(controlPoints))
    {
        int n = (int)ctrl_.size();
        if ((int)knots_.size() != n + deg_ + 1)
            throw std::invalid_argument("BSplineCurve3: bad knot vector size");
    }

    static std::vector<double> uniformKnots(int n, int p)
    {
        int m = n + p + 1;
        std::vector<double> U(m);
        for (int i = 0; i <= p;  ++i) U[i] = 0.0;
        for (int i = p+1; i < n; ++i) U[i] = double(i-p) / double(n-p);
        for (int i = n; i < m;   ++i) U[i] = 1.0;
        return U;
    }

    CurvePoint3 evaluate(double t) const override
    {
        int n = (int)ctrl_.size() - 1;
        int span = detail::bsplFindSpan(n, deg_, t, knots_);

        auto N   = detail::bsplBasis(span, t, deg_,   knots_);
        auto Nd  = (deg_ >= 1) ? detail::bsplBasis(span, t, deg_-1, knots_)
                               : std::vector<double>(1, 0.0);
        auto Ndd = (deg_ >= 2) ? detail::bsplBasis(span, t, deg_-2, knots_)
                               : std::vector<double>(1, 0.0);

        auto dN  = detail::bsplDeriv1(span, deg_, knots_, Nd);
        auto ddN = detail::bsplDeriv2(span, deg_, knots_, Ndd);

        CurvePoint3 cp;
        for (int k = 0; k <= deg_; ++k) {
            int ci = span - deg_ + k;
            cp.p  = cp.p  + ctrl_[ci] * N[k];
            cp.d1 = cp.d1 + ctrl_[ci] * dN[k];
            cp.d2 = cp.d2 + ctrl_[ci] * ddN[k];
        }
        return cp;
    }

    Interval domain() const override {
        return Interval{knots_[deg_], knots_[(int)ctrl_.size()]};
    }

    bool isClosed() const override {
        int n = (int)ctrl_.size();
        if (n < deg_ + 1) return false;
        return ctrl_.front().fuzzyEquals(ctrl_.back(), 1e-10);
    }

    BSplineCurve3 insertKnot(double t) const
    {
        int n = (int)ctrl_.size() - 1;
        int span = detail::bsplFindSpan(n, deg_, t, knots_);

        std::vector<double> newKnots = knots_;
        newKnots.insert(newKnots.begin() + span + 1, t);

        int nn = n + 1;
        std::vector<Vec3> newCtrl(nn + 1);
        for (int i = 0; i <= span - deg_; ++i) newCtrl[i] = ctrl_[i];
        for (int i = span - deg_ + 1; i <= span; ++i) {
            double denom = knots_[i + deg_] - knots_[i];
            double alpha = (denom > 0.0 || denom < 0.0) ? (t - knots_[i]) / denom : 0.0;
            newCtrl[i] = ctrl_[i-1] * (1.0 - alpha) + ctrl_[i] * alpha;
        }
        for (int i = span + 1; i <= nn; ++i) newCtrl[i] = ctrl_[i-1];

        return BSplineCurve3(deg_, std::move(newKnots), std::move(newCtrl));
    }

    int degree() const { return deg_; }
    const std::vector<double>& knots() const { return knots_; }
    const std::vector<Vec3>& controlPoints() const { return ctrl_; }

private:
    int deg_;
    std::vector<double> knots_;
    std::vector<Vec3> ctrl_;
};

// ── BSplineCurve2 ─────────────────────────────────────────────────────────────

class BSplineCurve2 : public ICurve2 {
public:
    BSplineCurve2(int degree, std::vector<double> knots,
                  std::vector<Vec2> controlPoints)
        : deg_(degree)
        , knots_(std::move(knots))
        , ctrl_(std::move(controlPoints))
    {
        int n = (int)ctrl_.size();
        if ((int)knots_.size() != n + deg_ + 1)
            throw std::invalid_argument("BSplineCurve2: bad knot vector size");
    }

    static std::vector<double> uniformKnots(int n, int p)
    {
        return BSplineCurve3::uniformKnots(n, p);
    }

    CurvePoint2 evaluate(double t) const override
    {
        int n = (int)ctrl_.size() - 1;
        int span = detail::bsplFindSpan(n, deg_, t, knots_);

        auto N   = detail::bsplBasis(span, t, deg_,   knots_);
        auto Nd  = (deg_ >= 1) ? detail::bsplBasis(span, t, deg_-1, knots_)
                               : std::vector<double>(1, 0.0);
        auto Ndd = (deg_ >= 2) ? detail::bsplBasis(span, t, deg_-2, knots_)
                               : std::vector<double>(1, 0.0);

        auto dN  = detail::bsplDeriv1(span, deg_, knots_, Nd);
        auto ddN = detail::bsplDeriv2(span, deg_, knots_, Ndd);

        CurvePoint2 cp;
        for (int k = 0; k <= deg_; ++k) {
            int ci = span - deg_ + k;
            cp.p  = cp.p  + ctrl_[ci] * N[k];
            cp.d1 = cp.d1 + ctrl_[ci] * dN[k];
            cp.d2 = cp.d2 + ctrl_[ci] * ddN[k];
        }
        return cp;
    }

    Interval domain() const override {
        return Interval{knots_[deg_], knots_[(int)ctrl_.size()]};
    }

    bool isClosed() const override {
        int n = (int)ctrl_.size();
        if (n < deg_ + 1) return false;
        return ctrl_.front().fuzzyEquals(ctrl_.back(), 1e-10);
    }

    BSplineCurve2 insertKnot(double t) const
    {
        int n = (int)ctrl_.size() - 1;
        int span = detail::bsplFindSpan(n, deg_, t, knots_);

        std::vector<double> newKnots = knots_;
        newKnots.insert(newKnots.begin() + span + 1, t);

        int nn = n + 1;
        std::vector<Vec2> newCtrl(nn + 1);
        for (int i = 0; i <= span - deg_; ++i) newCtrl[i] = ctrl_[i];
        for (int i = span - deg_ + 1; i <= span; ++i) {
            double denom = knots_[i + deg_] - knots_[i];
            double alpha = (denom > 0.0 || denom < 0.0) ? (t - knots_[i]) / denom : 0.0;
            newCtrl[i] = ctrl_[i-1] * (1.0 - alpha) + ctrl_[i] * alpha;
        }
        for (int i = span + 1; i <= nn; ++i) newCtrl[i] = ctrl_[i-1];

        return BSplineCurve2(deg_, std::move(newKnots), std::move(newCtrl));
    }

    int degree() const { return deg_; }
    const std::vector<double>& knots() const { return knots_; }
    const std::vector<Vec2>& controlPoints() const { return ctrl_; }

private:
    int deg_;
    std::vector<double> knots_;
    std::vector<Vec2> ctrl_;
};

} // namespace gk
