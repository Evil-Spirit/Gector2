#pragma once

#include "gk/math/Tolerances.h"
#include "gk/math/Vec3.h"
#include <cmath>
#include <string>

namespace gk {

/// Row-major 3×3 matrix.
struct Mat3x3
{
    double d[3][3]{};

    // ── Construction ────────────────────────────────────────────────────────
    constexpr Mat3x3() noexcept = default;
    constexpr Mat3x3(double a00, double a01, double a02,
                     double a10, double a11, double a12,
                     double a20, double a21, double a22) noexcept
    {
        d[0][0]=a00; d[0][1]=a01; d[0][2]=a02;
        d[1][0]=a10; d[1][1]=a11; d[1][2]=a12;
        d[2][0]=a20; d[2][1]=a21; d[2][2]=a22;
    }

    static constexpr Mat3x3 zero()     noexcept { return {0,0,0, 0,0,0, 0,0,0}; }
    static constexpr Mat3x3 identity() noexcept { return {1,0,0, 0,1,0, 0,0,1}; }

    /// Rotation about an arbitrary unit axis by angle (radians).
    static Mat3x3 rotationAxisAngle(const Vec3& axis, double angle) noexcept
    {
        double c = std::cos(angle);
        double s = std::sin(angle);
        double t = 1.0 - c;
        double x = axis.x, y = axis.y, z = axis.z;
        return {
            t*x*x + c,   t*x*y - s*z, t*x*z + s*y,
            t*x*y + s*z, t*y*y + c,   t*y*z - s*x,
            t*x*z - s*y, t*y*z + s*x, t*z*z + c
        };
    }

    // ── Element access ───────────────────────────────────────────────────────
    constexpr double  operator()(int r, int c) const noexcept { return d[r][c]; }
    constexpr double& operator()(int r, int c)       noexcept { return d[r][c]; }

    // ── Arithmetic ──────────────────────────────────────────────────────────
    constexpr Mat3x3 operator+(const Mat3x3& o) const noexcept
    {
        return { d[0][0]+o.d[0][0], d[0][1]+o.d[0][1], d[0][2]+o.d[0][2],
                 d[1][0]+o.d[1][0], d[1][1]+o.d[1][1], d[1][2]+o.d[1][2],
                 d[2][0]+o.d[2][0], d[2][1]+o.d[2][1], d[2][2]+o.d[2][2] };
    }
    constexpr Mat3x3 operator-(const Mat3x3& o) const noexcept
    {
        return { d[0][0]-o.d[0][0], d[0][1]-o.d[0][1], d[0][2]-o.d[0][2],
                 d[1][0]-o.d[1][0], d[1][1]-o.d[1][1], d[1][2]-o.d[1][2],
                 d[2][0]-o.d[2][0], d[2][1]-o.d[2][1], d[2][2]-o.d[2][2] };
    }
    constexpr Mat3x3 operator*(double s) const noexcept
    {
        return { d[0][0]*s, d[0][1]*s, d[0][2]*s,
                 d[1][0]*s, d[1][1]*s, d[1][2]*s,
                 d[2][0]*s, d[2][1]*s, d[2][2]*s };
    }
    friend constexpr Mat3x3 operator*(double s, const Mat3x3& m) noexcept { return m * s; }

    constexpr Mat3x3 operator*(const Mat3x3& o) const noexcept
    {
        Mat3x3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                r.d[i][j] = 0.0;
                for (int k = 0; k < 3; ++k)
                    r.d[i][j] += d[i][k] * o.d[k][j];
            }
        return r;
    }

    constexpr Vec3 operator*(const Vec3& v) const noexcept
    {
        return { d[0][0]*v.x + d[0][1]*v.y + d[0][2]*v.z,
                 d[1][0]*v.x + d[1][1]*v.y + d[1][2]*v.z,
                 d[2][0]*v.x + d[2][1]*v.y + d[2][2]*v.z };
    }

    // ── Linear algebra ───────────────────────────────────────────────────────
    constexpr Mat3x3 transposed() const noexcept
    {
        return { d[0][0], d[1][0], d[2][0],
                 d[0][1], d[1][1], d[2][1],
                 d[0][2], d[1][2], d[2][2] };
    }

    constexpr double determinant() const noexcept
    {
        return d[0][0] * (d[1][1]*d[2][2] - d[1][2]*d[2][1])
             - d[0][1] * (d[1][0]*d[2][2] - d[1][2]*d[2][0])
             + d[0][2] * (d[1][0]*d[2][1] - d[1][1]*d[2][0]);
    }

    /// Returns the inverse, or zero() if the matrix is singular.
    Mat3x3 inverse(double tol = kDefaultTolerance) const noexcept
    {
        double det = determinant();
        if (det > -tol && det < tol) return zero();
        double invDet = 1.0 / det;
        return Mat3x3{
            (d[1][1]*d[2][2] - d[1][2]*d[2][1]) * invDet,
            (d[0][2]*d[2][1] - d[0][1]*d[2][2]) * invDet,
            (d[0][1]*d[1][2] - d[0][2]*d[1][1]) * invDet,

            (d[1][2]*d[2][0] - d[1][0]*d[2][2]) * invDet,
            (d[0][0]*d[2][2] - d[0][2]*d[2][0]) * invDet,
            (d[0][2]*d[1][0] - d[0][0]*d[1][2]) * invDet,

            (d[1][0]*d[2][1] - d[1][1]*d[2][0]) * invDet,
            (d[0][1]*d[2][0] - d[0][0]*d[2][1]) * invDet,
            (d[0][0]*d[1][1] - d[0][1]*d[1][0]) * invDet
        };
    }

    // ── Comparison ──────────────────────────────────────────────────────────
    bool fuzzyEquals(const Mat3x3& o, double tol = kDefaultTolerance) const noexcept
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                if (!fuzzyEqual(d[r][c], o.d[r][c], tol)) return false;
        return true;
    }
    constexpr bool operator==(const Mat3x3& o) const noexcept
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                if (d[r][c] != o.d[r][c]) return false;
        return true;
    }
    constexpr bool operator!=(const Mat3x3& o) const noexcept { return !(*this == o); }

    // ── Serialization ───────────────────────────────────────────────────────
    std::string toString() const
    {
        std::string s = "[[";
        for (int r = 0; r < 3; ++r)
        {
            if (r) s += ",[";
            for (int c = 0; c < 3; ++c)
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
