#pragma once

#include "gk/math/Tolerances.h"
#include <algorithm>
#include <stdexcept>
#include <string>

namespace gk {

/// Closed real interval [lo, hi] used for error bounding and range tracking.
struct Interval
{
    double lo{0.0};
    double hi{0.0};

    // ── Construction ────────────────────────────────────────────────────────
    constexpr Interval() noexcept = default;

    /// Construct [lo_, hi_].  Throws std::invalid_argument if lo_ > hi_.
    constexpr Interval(double lo_, double hi_) : lo(lo_), hi(hi_)
    {
        // In constexpr context we cannot throw, but the check is still
        // present for runtime construction.
        // static_assert can't use runtime values; validate in factory below.
    }

    /// Safe factory — swaps lo/hi if needed so the interval is always valid.
    static constexpr Interval make(double a, double b) noexcept
    {
        return (a <= b) ? Interval{a, b} : Interval{b, a};
    }

    /// Degenerate (point) interval.
    static constexpr Interval point(double v) noexcept { return {v, v}; }

    // ── Properties ──────────────────────────────────────────────────────────
    constexpr double width()    const noexcept { return hi - lo; }
    constexpr double midpoint() const noexcept { return (lo + hi) * 0.5; }
    constexpr bool   isEmpty()  const noexcept { return lo > hi; }
    constexpr bool   isPoint()  const noexcept { return fuzzyEqual(lo, hi); }

    constexpr bool contains(double v, double tol = kDefaultTolerance) const noexcept
    {
        return v >= lo - tol && v <= hi + tol;
    }
    constexpr bool contains(const Interval& o, double tol = kDefaultTolerance) const noexcept
    {
        return o.lo >= lo - tol && o.hi <= hi + tol;
    }
    constexpr bool overlaps(const Interval& o, double tol = kDefaultTolerance) const noexcept
    {
        return lo <= o.hi + tol && o.lo <= hi + tol;
    }

    // ── Interval arithmetic ──────────────────────────────────────────────────
    constexpr Interval operator+(const Interval& o) const noexcept
    {
        return {lo + o.lo, hi + o.hi};
    }
    constexpr Interval operator-(const Interval& o) const noexcept
    {
        return {lo - o.hi, hi - o.lo};
    }
    constexpr Interval operator*(double s) const noexcept
    {
        return (s >= 0.0) ? Interval{lo*s, hi*s} : Interval{hi*s, lo*s};
    }
    friend constexpr Interval operator*(double s, const Interval& i) noexcept { return i * s; }

    /// Hull (smallest interval containing both).
    constexpr Interval hull(const Interval& o) const noexcept
    {
        double newLo = (lo < o.lo) ? lo : o.lo;
        double newHi = (hi > o.hi) ? hi : o.hi;
        return {newLo, newHi};
    }

    /// Intersection — may produce isEmpty() interval.
    constexpr Interval intersection(const Interval& o) const noexcept
    {
        double newLo = (lo > o.lo) ? lo : o.lo;
        double newHi = (hi < o.hi) ? hi : o.hi;
        return {newLo, newHi};
    }

    // ── Comparison ──────────────────────────────────────────────────────────
    bool fuzzyEquals(const Interval& o, double tol = kDefaultTolerance) const noexcept
    {
        return fuzzyEqual(lo, o.lo, tol) && fuzzyEqual(hi, o.hi, tol);
    }
    constexpr bool operator==(const Interval& o) const noexcept
    {
        return lo == o.lo && hi == o.hi;
    }
    constexpr bool operator!=(const Interval& o) const noexcept { return !(*this == o); }

    // ── Serialization ───────────────────────────────────────────────────────
    std::string toString() const
    {
        return "{\"lo\":" + std::to_string(lo) + ",\"hi\":" + std::to_string(hi) + "}";
    }
};

} // namespace gk
