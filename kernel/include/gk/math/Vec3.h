#pragma once

#include "gk/math/Tolerances.h"
#include <cmath>
#include <string>

namespace gk {

/// 3-D column vector.
struct Vec3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};

    // ── Construction ────────────────────────────────────────────────────────
    constexpr Vec3() noexcept = default;
    constexpr Vec3(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}

    static constexpr Vec3 zero()  noexcept { return {0.0, 0.0, 0.0}; }
    static constexpr Vec3 unitX() noexcept { return {1.0, 0.0, 0.0}; }
    static constexpr Vec3 unitY() noexcept { return {0.0, 1.0, 0.0}; }
    static constexpr Vec3 unitZ() noexcept { return {0.0, 0.0, 1.0}; }

    // ── Arithmetic ──────────────────────────────────────────────────────────
    constexpr Vec3 operator+(const Vec3& o) const noexcept { return {x+o.x, y+o.y, z+o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const noexcept { return {x-o.x, y-o.y, z-o.z}; }
    constexpr Vec3 operator*(double s)      const noexcept { return {x*s,   y*s,   z*s};   }
    constexpr Vec3 operator/(double s)      const noexcept { return {x/s,   y/s,   z/s};   }
    constexpr Vec3 operator-()              const noexcept { return {-x, -y, -z};           }

    friend constexpr Vec3 operator*(double s, const Vec3& v) noexcept { return v * s; }

    constexpr Vec3& operator+=(const Vec3& o) noexcept { x+=o.x; y+=o.y; z+=o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) noexcept { x-=o.x; y-=o.y; z-=o.z; return *this; }
    constexpr Vec3& operator*=(double s)      noexcept { x*=s;   y*=s;   z*=s;   return *this; }
    constexpr Vec3& operator/=(double s)      noexcept { x/=s;   y/=s;   z/=s;   return *this; }

    // ── Geometry ────────────────────────────────────────────────────────────
    constexpr double dot(const Vec3& o) const noexcept
    {
        return x * o.x + y * o.y + z * o.z;
    }
    constexpr Vec3 cross(const Vec3& o) const noexcept
    {
        return { y * o.z - z * o.y,
                 z * o.x - x * o.z,
                 x * o.y - y * o.x };
    }
    constexpr double squaredNorm() const noexcept { return dot(*this); }
    double           norm()        const noexcept { return std::sqrt(squaredNorm()); }
    Vec3             normalized()  const noexcept
    {
        double n = norm();
        return (n > 0.0) ? (*this / n) : zero();
    }

    // ── Comparison ──────────────────────────────────────────────────────────
    bool fuzzyEquals(const Vec3& o, double tol = kDefaultTolerance) const noexcept
    {
        return fuzzyEqual(x, o.x, tol) && fuzzyEqual(y, o.y, tol) && fuzzyEqual(z, o.z, tol);
    }
    constexpr bool operator==(const Vec3& o) const noexcept
    {
        return x == o.x && y == o.y && z == o.z;
    }
    constexpr bool operator!=(const Vec3& o) const noexcept { return !(*this == o); }

    // ── Serialization ───────────────────────────────────────────────────────
    std::string toString() const
    {
        return "{\"x\":" + std::to_string(x) +
               ",\"y\":" + std::to_string(y) +
               ",\"z\":" + std::to_string(z) + "}";
    }
};

} // namespace gk
