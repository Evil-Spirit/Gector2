#pragma once
// Iteration 1.3 — Tolerance & Precision Model
//
// Provides:
//   * ToleranceSet  — a named tuple of linear/angular/parametric tolerances.
//   * Tolerances    — global singleton with a per-thread push/pop stack so that
//                     a section of code can temporarily tighten or loosen limits.
//   * ToleranceGuard— RAII wrapper around Tolerances::push/pop.
//   * FuzzyCompare  — three comparison strategies usable at call-site:
//                       Absolute  |a-b| <= tol
//                       Relative  |a-b| / max(|a|,|b|,1) <= tol
//                       Ulp       bit-distance between IEEE-754 doubles

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace gk {

// ── Tolerance set ─────────────────────────────────────────────────────────────

/// A named collection of tolerances used throughout the kernel.
struct ToleranceSet
{
    double linear    = 1e-7;   ///< Cartesian distance tolerance (mm or model units).
    double angular   = 1e-9;   ///< Angular tolerance (radians).
    double parametric = 1e-10; ///< UV / parametric-space tolerance.
};

// ── Tolerances singleton ──────────────────────────────────────────────────────

/// Global tolerance manager.
/// Uses a per-thread stack so nested sessions can override tolerances locally.
class Tolerances
{
public:
    /// Returns the tolerance set at the top of the current thread's stack.
    static const ToleranceSet& current() noexcept
    {
        return stack().back();
    }

    /// Push a new tolerance set onto the stack. Pair with pop() or use ToleranceGuard.
    static void push(const ToleranceSet& t)
    {
        stack().push_back(t);
    }

    /// Pop the top-most tolerance override.  At least one entry must remain.
    static void pop() noexcept
    {
        auto& s = stack();
        if (s.size() > 1)
            s.pop_back();
    }

    // ── Convenience accessors ─────────────────────────────────────────────────
    static double linear()     noexcept { return current().linear;     }
    static double angular()    noexcept { return current().angular;    }
    static double parametric() noexcept { return current().parametric; }

private:
    /// Per-thread stack; always contains at least the default entry.
    static std::vector<ToleranceSet>& stack() noexcept
    {
        static thread_local std::vector<ToleranceSet> s{ToleranceSet{}};
        return s;
    }
};

// ── RAII tolerance guard ──────────────────────────────────────────────────────

/// Push a custom tolerance set for the lifetime of this guard, then restore.
class ToleranceGuard
{
public:
    explicit ToleranceGuard(const ToleranceSet& t) { Tolerances::push(t); }
    ~ToleranceGuard() noexcept { Tolerances::pop(); }

    // Non-copyable, non-movable.
    ToleranceGuard(const ToleranceGuard&) = delete;
    ToleranceGuard& operator=(const ToleranceGuard&) = delete;
};

// ── FuzzyCompare ──────────────────────────────────────────────────────────────

/// Comparison strategies that can be selected at call-site.
struct FuzzyCompare
{
    // ── Absolute ──────────────────────────────────────────────────────────────
    /// |a - b| <= tol
    static bool absolute(double a, double b, double tol) noexcept
    {
        double d = a - b;
        return (d >= -tol) && (d <= tol);
    }

    // ── Relative ──────────────────────────────────────────────────────────────
    /// |a - b| / max(|a|, |b|, 1.0) <= tol
    static bool relative(double a, double b, double tol) noexcept
    {
        double scale = std::max({std::abs(a), std::abs(b), 1.0});
        double d = (a - b) / scale;
        return (d >= -tol) && (d <= tol);
    }

    // ── ULP (Units in the Last Place) ─────────────────────────────────────────
    /// Bit-distance between the IEEE-754 representations of a and b.
    /// Returns true when the distance is <= maxUlps.
    ///
    /// Edge cases:
    ///   NaN : always false.
    ///   ±Inf: true only when both are the same infinity.
    ///   ±0  : treated as equal (0 ULP apart).
    static bool ulp(double a, double b, int maxUlps = 4) noexcept
    {
        if (std::isnan(a) || std::isnan(b))   return false;
        if (a == b)                            return true; // handles ±0, same ±Inf
        if (std::isinf(a) || std::isinf(b))   return false;

        // Reinterpret the bit patterns as signed 64-bit integers.
        std::int64_t ia = 0, ib = 0;
        std::memcpy(&ia, &a, sizeof(ia));
        std::memcpy(&ib, &b, sizeof(ib));

        // Convert sign-magnitude IEEE-754 representation to a linear integer
        // ordering so that adjacent floats differ by exactly 1.
        // For negative values: flip all bits except the sign bit.
        // The constant below is 2^63 as a signed int64 (i.e. INT64_MIN).
        static constexpr std::int64_t kSignBit = static_cast<std::int64_t>(
            std::uint64_t{1} << 63);
        if (ia < 0) ia = kSignBit - ia;
        if (ib < 0) ib = kSignBit - ib;

        std::int64_t diff = ia - ib;
        if (diff < 0) diff = -diff;
        return diff <= static_cast<std::int64_t>(maxUlps);
    }
};

} // namespace gk
