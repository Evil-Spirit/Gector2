#pragma once

#include "gk/math/Tolerances.h"
#include "gk/math/Vec3.h"
#include "gk/math/Mat3x3.h"
#include <cmath>
#include <string>

namespace gk {

/// Unit quaternion representing a 3-D rotation: q = w + xi + yj + zk.
/// Convention: the vector part is (x, y, z) and the scalar part is w.
struct Quaternion
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0}; ///< scalar part; identity quaternion has w=1.

    // ── Construction ────────────────────────────────────────────────────────
    constexpr Quaternion() noexcept = default;
    constexpr Quaternion(double x_, double y_, double z_, double w_) noexcept
        : x(x_), y(y_), z(z_), w(w_) {}

    /// Construct from a unit axis and an angle in radians.
    static Quaternion fromAxisAngle(const Vec3& axis, double angle) noexcept
    {
        double half = angle * 0.5;
        double s    = std::sin(half);
        return { axis.x * s, axis.y * s, axis.z * s, std::cos(half) };
    }

    static constexpr Quaternion identity() noexcept { return {0.0, 0.0, 0.0, 1.0}; }

    // ── Arithmetic ──────────────────────────────────────────────────────────
    /// Hamilton product.
    constexpr Quaternion operator*(const Quaternion& o) const noexcept
    {
        return {
            w*o.x + x*o.w + y*o.z - z*o.y,
            w*o.y - x*o.z + y*o.w + z*o.x,
            w*o.z + x*o.y - y*o.x + z*o.w,
            w*o.w - x*o.x - y*o.y - z*o.z
        };
    }

    constexpr Quaternion conjugate() const noexcept { return {-x, -y, -z, w}; }

    constexpr double squaredNorm() const noexcept
    {
        return x*x + y*y + z*z + w*w;
    }
    double norm() const noexcept { return std::sqrt(squaredNorm()); }

    Quaternion normalized() const noexcept
    {
        double n = norm();
        return (n > 0.0) ? Quaternion{x/n, y/n, z/n, w/n} : identity();
    }

    // ── Rotation ─────────────────────────────────────────────────────────────
    /// Rotate a vector by this quaternion (assumes unit quaternion).
    constexpr Vec3 rotate(const Vec3& v) const noexcept
    {
        // Optimised sandwich product: q * [0,v] * q^-1
        Vec3 qv{x, y, z};
        Vec3 t = 2.0 * qv.cross(v);
        return v + w * t + qv.cross(t);
    }

    /// Convert to an equivalent rotation matrix.
    Mat3x3 toMatrix() const noexcept
    {
        double x2=x*x, y2=y*y, z2=z*z;
        double xy=x*y, xz=x*z, yz=y*z;
        double wx=w*x, wy=w*y, wz=w*z;
        return {
            1-2*(y2+z2),  2*(xy-wz),   2*(xz+wy),
            2*(xy+wz),    1-2*(x2+z2), 2*(yz-wx),
            2*(xz-wy),    2*(yz+wx),   1-2*(x2+y2)
        };
    }

    // ── Comparison ──────────────────────────────────────────────────────────
    bool fuzzyEquals(const Quaternion& o, double tol = kDefaultTolerance) const noexcept
    {
        // Two unit quaternions represent the same rotation when q == o or q == -o.
        bool same = fuzzyEqual(x, o.x, tol) && fuzzyEqual(y, o.y, tol) &&
                    fuzzyEqual(z, o.z, tol) && fuzzyEqual(w, o.w, tol);
        bool neg  = fuzzyEqual(x,-o.x, tol) && fuzzyEqual(y,-o.y, tol) &&
                    fuzzyEqual(z,-o.z, tol) && fuzzyEqual(w,-o.w, tol);
        return same || neg;
    }
    constexpr bool operator==(const Quaternion& o) const noexcept
    {
        return x == o.x && y == o.y && z == o.z && w == o.w;
    }
    constexpr bool operator!=(const Quaternion& o) const noexcept { return !(*this == o); }

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
