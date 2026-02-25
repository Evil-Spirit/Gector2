#pragma once

#include "gk/math/Tolerances.h"
#include "gk/math/Vec4.h"
#include <string>

namespace gk {

/// Row-major 4×4 matrix (affine / projective transforms).
struct Mat4x4
{
    double d[4][4]{};

    // ── Construction ────────────────────────────────────────────────────────
    constexpr Mat4x4() noexcept = default;
    constexpr Mat4x4(double a00, double a01, double a02, double a03,
                     double a10, double a11, double a12, double a13,
                     double a20, double a21, double a22, double a23,
                     double a30, double a31, double a32, double a33) noexcept
    {
        d[0][0]=a00; d[0][1]=a01; d[0][2]=a02; d[0][3]=a03;
        d[1][0]=a10; d[1][1]=a11; d[1][2]=a12; d[1][3]=a13;
        d[2][0]=a20; d[2][1]=a21; d[2][2]=a22; d[2][3]=a23;
        d[3][0]=a30; d[3][1]=a31; d[3][2]=a32; d[3][3]=a33;
    }

    static constexpr Mat4x4 zero() noexcept
    {
        return {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
    }
    static constexpr Mat4x4 identity() noexcept
    {
        return {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    }

    // ── Element access ───────────────────────────────────────────────────────
    constexpr double  operator()(int r, int c) const noexcept { return d[r][c]; }
    constexpr double& operator()(int r, int c)       noexcept { return d[r][c]; }

    // ── Arithmetic ──────────────────────────────────────────────────────────
    constexpr Mat4x4 operator+(const Mat4x4& o) const noexcept
    {
        Mat4x4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.d[i][j] = d[i][j] + o.d[i][j];
        return r;
    }
    constexpr Mat4x4 operator-(const Mat4x4& o) const noexcept
    {
        Mat4x4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.d[i][j] = d[i][j] - o.d[i][j];
        return r;
    }
    constexpr Mat4x4 operator*(double s) const noexcept
    {
        Mat4x4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.d[i][j] = d[i][j] * s;
        return r;
    }
    friend constexpr Mat4x4 operator*(double s, const Mat4x4& m) noexcept { return m * s; }

    constexpr Mat4x4 operator*(const Mat4x4& o) const noexcept
    {
        Mat4x4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                r.d[i][j] = 0.0;
                for (int k = 0; k < 4; ++k)
                    r.d[i][j] += d[i][k] * o.d[k][j];
            }
        return r;
    }

    constexpr Vec4 operator*(const Vec4& v) const noexcept
    {
        return { d[0][0]*v.x + d[0][1]*v.y + d[0][2]*v.z + d[0][3]*v.w,
                 d[1][0]*v.x + d[1][1]*v.y + d[1][2]*v.z + d[1][3]*v.w,
                 d[2][0]*v.x + d[2][1]*v.y + d[2][2]*v.z + d[2][3]*v.w,
                 d[3][0]*v.x + d[3][1]*v.y + d[3][2]*v.z + d[3][3]*v.w };
    }

    // ── Linear algebra ───────────────────────────────────────────────────────
    constexpr Mat4x4 transposed() const noexcept
    {
        return { d[0][0], d[1][0], d[2][0], d[3][0],
                 d[0][1], d[1][1], d[2][1], d[3][1],
                 d[0][2], d[1][2], d[2][2], d[3][2],
                 d[0][3], d[1][3], d[2][3], d[3][3] };
    }

    // ── Comparison ──────────────────────────────────────────────────────────
    bool fuzzyEquals(const Mat4x4& o, double tol = kDefaultTolerance) const noexcept
    {
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                if (!fuzzyEqual(d[r][c], o.d[r][c], tol)) return false;
        return true;
    }
    constexpr bool operator==(const Mat4x4& o) const noexcept
    {
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                if (d[r][c] != o.d[r][c]) return false;
        return true;
    }
    constexpr bool operator!=(const Mat4x4& o) const noexcept { return !(*this == o); }

    // ── Serialization ───────────────────────────────────────────────────────
    std::string toString() const
    {
        std::string s = "[[";
        for (int r = 0; r < 4; ++r)
        {
            if (r) s += ",[";
            for (int c = 0; c < 4; ++c)
            {
                if (c) s += ",";
                s += std::to_string(d[r][c]);
            }
            s += "]";
        }
        return s + "]";
    }
};

} // namespace gk
