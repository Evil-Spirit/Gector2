#pragma once

#include "gk/math/Tolerances.h"
#include <cmath>
#include <string>

namespace gk {

/// 4-D column vector (homogeneous coordinates / SIMD-friendly padding).
struct Vec4
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{0.0};

    // ── Construction ────────────────────────────────────────────────────────
    constexpr Vec4() noexcept = default;
    constexpr Vec4(double x_, double y_, double z_, double w_) noexcept
        : x(x_), y(y_), z(z_), w(w_) {}

    static constexpr Vec4 zero()  noexcept { return {0.0, 0.0, 0.0, 0.0}; }
    static constexpr Vec4 unitX() noexcept { return {1.0, 0.0, 0.0, 0.0}; }
    static constexpr Vec4 unitY() noexcept { return {0.0, 1.0, 0.0, 0.0}; }
    static constexpr Vec4 unitZ() noexcept { return {0.0, 0.0, 1.0, 0.0}; }
    static constexpr Vec4 unitW() noexcept { return {0.0, 0.0, 0.0, 1.0}; }

    // ── Arithmetic ──────────────────────────────────────────────────────────
    constexpr Vec4 operator+(const Vec4& o) const noexcept
    {
        return {x+o.x, y+o.y, z+o.z, w+o.w};
    }
    constexpr Vec4 operator-(const Vec4& o) const noexcept
    {
        return {x-o.x, y-o.y, z-o.z, w-o.w};
    }
    constexpr Vec4 operator*(double s) const noexcept { return {x*s, y*s, z*s, w*s}; }
    constexpr Vec4 operator/(double s) const noexcept { return {x/s, y/s, z/s, w/s}; }
    constexpr Vec4 operator-()         const noexcept { return {-x, -y, -z, -w};      }

    friend constexpr Vec4 operator*(double s, const Vec4& v) noexcept { return v * s; }

    constexpr Vec4& operator+=(const Vec4& o) noexcept
    {
        x+=o.x; y+=o.y; z+=o.z; w+=o.w; return *this;
    }
    constexpr Vec4& operator-=(const Vec4& o) noexcept
    {
        x-=o.x; y-=o.y; z-=o.z; w-=o.w; return *this;
    }
    constexpr Vec4& operator*=(double s) noexcept { x*=s; y*=s; z*=s; w*=s; return *this; }
    constexpr Vec4& operator/=(double s) noexcept { x/=s; y/=s; z/=s; w/=s; return *this; }

    // ── Geometry ────────────────────────────────────────────────────────────
    constexpr double dot(const Vec4& o) const noexcept
    {
        return x*o.x + y*o.y + z*o.z + w*o.w;
    }
    constexpr double squaredNorm() const noexcept { return dot(*this); }
    double           norm()        const noexcept { return std::sqrt(squaredNorm()); }
    Vec4             normalized()  const noexcept
    {
        double n = norm();
        return (n > 0.0) ? (*this / n) : zero();
    }

    // ── Comparison ──────────────────────────────────────────────────────────
    bool fuzzyEquals(const Vec4& o, double tol = kDefaultTolerance) const noexcept
    {
        return fuzzyEqual(x, o.x, tol) && fuzzyEqual(y, o.y, tol) &&
               fuzzyEqual(z, o.z, tol) && fuzzyEqual(w, o.w, tol);
    }
    constexpr bool operator==(const Vec4& o) const noexcept
    {
        return x == o.x && y == o.y && z == o.z && w == o.w;
    }
    constexpr bool operator!=(const Vec4& o) const noexcept { return !(*this == o); }

    // ── Serialization ───────────────────────────────────────────────────────
    std::string toString() const
    {
        return "{\"x\":" + std::to_string(x) +
               ",\"y\":" + std::to_string(y) +
               ",\"z\":" + std::to_string(z) +
               ",\"w\":" + std::to_string(w) + "}";
    }
};

} // namespace gk
