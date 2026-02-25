#pragma once

#include "gk/math/Tolerances.h"
#include "gk/math/Vec2.h"
#include <string>

namespace gk {

/// Row-major 2×2 matrix.
/// Element access: m[row][col].
struct Mat2x2
{
    double d[2][2]{};

    // ── Construction ────────────────────────────────────────────────────────
    constexpr Mat2x2() noexcept = default;
    constexpr Mat2x2(double a00, double a01,
                     double a10, double a11) noexcept
    {
        d[0][0]=a00; d[0][1]=a01;
        d[1][0]=a10; d[1][1]=a11;
    }

    static constexpr Mat2x2 zero() noexcept { return {0,0,0,0}; }
    static constexpr Mat2x2 identity() noexcept { return {1,0,0,1}; }

    // ── Element access ───────────────────────────────────────────────────────
    constexpr double  operator()(int r, int c) const noexcept { return d[r][c]; }
    constexpr double& operator()(int r, int c)       noexcept { return d[r][c]; }

    // ── Arithmetic ──────────────────────────────────────────────────────────
    constexpr Mat2x2 operator+(const Mat2x2& o) const noexcept
    {
        return { d[0][0]+o.d[0][0], d[0][1]+o.d[0][1],
                 d[1][0]+o.d[1][0], d[1][1]+o.d[1][1] };
    }
    constexpr Mat2x2 operator-(const Mat2x2& o) const noexcept
    {
        return { d[0][0]-o.d[0][0], d[0][1]-o.d[0][1],
                 d[1][0]-o.d[1][0], d[1][1]-o.d[1][1] };
    }
    constexpr Mat2x2 operator*(double s) const noexcept
    {
        return { d[0][0]*s, d[0][1]*s,
                 d[1][0]*s, d[1][1]*s };
    }
    friend constexpr Mat2x2 operator*(double s, const Mat2x2& m) noexcept { return m * s; }

    constexpr Mat2x2 operator*(const Mat2x2& o) const noexcept
    {
        return {
            d[0][0]*o.d[0][0] + d[0][1]*o.d[1][0],
            d[0][0]*o.d[0][1] + d[0][1]*o.d[1][1],
            d[1][0]*o.d[0][0] + d[1][1]*o.d[1][0],
            d[1][0]*o.d[0][1] + d[1][1]*o.d[1][1]
        };
    }

    constexpr Vec2 operator*(const Vec2& v) const noexcept
    {
        return { d[0][0]*v.x + d[0][1]*v.y,
                 d[1][0]*v.x + d[1][1]*v.y };
    }

    // ── Linear algebra ───────────────────────────────────────────────────────
    constexpr Mat2x2 transposed() const noexcept
    {
        return { d[0][0], d[1][0],
                 d[0][1], d[1][1] };
    }
    constexpr double determinant() const noexcept
    {
        return d[0][0]*d[1][1] - d[0][1]*d[1][0];
    }
    /// Returns the inverse, or zero() if the matrix is singular (|det| < tol).
    Mat2x2 inverse(double tol = kDefaultTolerance) const noexcept
    {
        double det = determinant();
        if (det > -tol && det < tol) return zero();
        return Mat2x2{ d[1][1], -d[0][1], -d[1][0], d[0][0] } * (1.0 / det);
    }

    // ── Comparison ──────────────────────────────────────────────────────────
    bool fuzzyEquals(const Mat2x2& o, double tol = kDefaultTolerance) const noexcept
    {
        for (int r = 0; r < 2; ++r)
            for (int c = 0; c < 2; ++c)
                if (!fuzzyEqual(d[r][c], o.d[r][c], tol)) return false;
        return true;
    }
    constexpr bool operator==(const Mat2x2& o) const noexcept
    {
        for (int r = 0; r < 2; ++r)
            for (int c = 0; c < 2; ++c)
                if (d[r][c] != o.d[r][c]) return false;
        return true;
    }
    constexpr bool operator!=(const Mat2x2& o) const noexcept { return !(*this == o); }

    // ── Serialization ───────────────────────────────────────────────────────
    std::string toString() const
    {
        return std::string("[[") +
               std::to_string(d[0][0]) + "," + std::to_string(d[0][1]) + "],[" +
               std::to_string(d[1][0]) + "," + std::to_string(d[1][1]) + "]]";
    }
};

} // namespace gk
