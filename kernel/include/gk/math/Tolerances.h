#pragma once

#include <cmath>
#include <string>

namespace gk {

/// Tolerance used by fuzzy-equality helpers when no explicit tolerance is given.
static constexpr double kDefaultTolerance = 1e-10;

/// Returns true when |a - b| <= tol.
inline constexpr bool fuzzyEqual(double a, double b, double tol = kDefaultTolerance) noexcept
{
    double d = a - b;
    return (d >= -tol) && (d <= tol);
}

} // namespace gk
