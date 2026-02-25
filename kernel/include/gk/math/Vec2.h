#pragma once

#include "gk/math/Tolerances.h"
#include <cmath>
#include <string>

namespace gk {

/// 2-D column vector.
struct Vec2
{
    double x{0.0};
    double y{0.0};

    // ── Construction ────────────────────────────────────────────────────────
    constexpr Vec2() noexcept = default;
    constexpr Vec2(double x_, double y_) noexcept : x(x_), y(y_) {}

    static constexpr Vec2 zero()  noexcept { return {0.0, 0.0}; }
    static constexpr Vec2 unitX() noexcept { return {1.0, 0.0}; }
    static constexpr Vec2 unitY() noexcept { return {0.0, 1.0}; }

    // ── Arithmetic ──────────────────────────────────────────────────────────
    constexpr Vec2 operator+(const Vec2& o) const noexcept { return {x + o.x, y + o.y}; }
    constexpr Vec2 operator-(const Vec2& o) const noexcept { return {x - o.x, y - o.y}; }
    constexpr Vec2 operator*(double s)      const noexcept { return {x * s,   y * s};   }
    constexpr Vec2 operator/(double s)      const noexcept { return {x / s,   y / s};   }
    constexpr Vec2 operator-()              const noexcept { return {-x, -y};            }

    friend constexpr Vec2 operator*(double s, const Vec2& v) noexcept { return v * s; }

    constexpr Vec2& operator+=(const Vec2& o) noexcept { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(const Vec2& o) noexcept { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(double s)      noexcept { x *= s;   y *= s;   return *this; }
    constexpr Vec2& operator/=(double s)      noexcept { x /= s;   y /= s;   return *this; }

    // ── Geometry ────────────────────────────────────────────────────────────
    constexpr double dot(const Vec2& o)   const noexcept { return x * o.x + y * o.y; }
    /// Scalar 2-D cross product (z-component of the 3-D cross product).
    constexpr double cross(const Vec2& o) const noexcept { return x * o.y - y * o.x; }
    constexpr double squaredNorm()        const noexcept { return dot(*this); }
    double           norm()               const noexcept { return std::sqrt(squaredNorm()); }
    Vec2             normalized()         const noexcept
    {
        double n = norm();
        return (n > 0.0) ? (*this / n) : zero();
    }

    // ── Comparison ──────────────────────────────────────────────────────────
    bool fuzzyEquals(const Vec2& o, double tol = kDefaultTolerance) const noexcept
    {
        return fuzzyEqual(x, o.x, tol) && fuzzyEqual(y, o.y, tol);
    }
    constexpr bool operator==(const Vec2& o) const noexcept { return x == o.x && y == o.y; }
    constexpr bool operator!=(const Vec2& o) const noexcept { return !(*this == o); }

    // ── Serialization ───────────────────────────────────────────────────────
    std::string toString() const
    {
        return "{\"x\":" + std::to_string(x) + ",\"y\":" + std::to_string(y) + "}";
    }
};

} // namespace gk
