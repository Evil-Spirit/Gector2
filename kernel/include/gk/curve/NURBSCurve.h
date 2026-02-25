#pragma once
#include "gk/curve/BSplineCurve.h"
#include <cmath>
#include <stdexcept>
#include <vector>

namespace gk {

struct Vec4H {
    double x{0}, y{0}, z{0}, w{0};
    Vec4H() = default;
    Vec4H(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
    Vec4H operator+(const Vec4H& o) const { return {x+o.x, y+o.y, z+o.z, w+o.w}; }
    Vec4H operator*(double s)       const { return {x*s, y*s, z*s, w*s}; }
    friend Vec4H operator*(double s, const Vec4H& v) { return v * s; }
};

class NURBSCurve3 : public ICurve3 {
public:
    NURBSCurve3(int degree, std::vector<double> knots,
                std::vector<Vec3> controlPoints,
                std::vector<double> weights)
        : deg_(degree)
        , knots_(std::move(knots))
        , ctrl_(std::move(controlPoints))
        , weights_(std::move(weights))
    {
        int n = (int)ctrl_.size();
        if ((int)knots_.size() != n + deg_ + 1)
            throw std::invalid_argument("NURBSCurve3: bad knot vector size");
        if ((int)weights_.size() != n)
            throw std::invalid_argument("NURBSCurve3: weights size mismatch");
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

        Vec4H Hw, dHw, ddHw;
        for (int k = 0; k <= deg_; ++k) {
            int ci = span - deg_ + k;
            double w = weights_[ci];
            Vec4H hw(ctrl_[ci].x * w, ctrl_[ci].y * w, ctrl_[ci].z * w, w);
            Hw   = Hw   + hw * N[k];
            dHw  = dHw  + hw * dN[k];
            ddHw = ddHw + hw * ddN[k];
        }

        double W = Hw.w;
        if (W == 0.0) {
            return CurvePoint3{Vec3::zero(), Vec3::zero(), Vec3::zero()};
        }
        double Winv = 1.0 / W;

        Vec3 P(Hw.x * Winv, Hw.y * Winv, Hw.z * Winv);

        double dW = dHw.w;
        Vec3 d1((dHw.x - dW * P.x) * Winv,
                (dHw.y - dW * P.y) * Winv,
                (dHw.z - dW * P.z) * Winv);

        double ddW = ddHw.w;
        Vec3 d2((ddHw.x - ddW * P.x - 2.0 * dW * d1.x) * Winv,
                (ddHw.y - ddW * P.y - 2.0 * dW * d1.y) * Winv,
                (ddHw.z - ddW * P.z - 2.0 * dW * d1.z) * Winv);

        return CurvePoint3{P, d1, d2};
    }

    Interval domain() const override {
        return Interval{knots_[deg_], knots_[(int)ctrl_.size()]};
    }

    bool isClosed() const override {
        return ctrl_.front().fuzzyEquals(ctrl_.back(), 1e-10);
    }

    int degree() const { return deg_; }
    const std::vector<double>& knots() const { return knots_; }
    const std::vector<Vec3>& controlPoints() const { return ctrl_; }
    const std::vector<double>& weights() const { return weights_; }

private:
    int deg_;
    std::vector<double> knots_;
    std::vector<Vec3> ctrl_;
    std::vector<double> weights_;
};

} // namespace gk
