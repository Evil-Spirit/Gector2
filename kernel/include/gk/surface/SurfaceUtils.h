#pragma once
// Iteration 3.4 — Surface utility algorithms.
//
// Provides:
//   * gaussianCurvature / meanCurvature  — computed from any ISurface via the
//     first and second fundamental forms.
//   * SurfaceMesh   — indexed triangle mesh returned by tessellate().
//   * tessellate()  — uniform grid tessellation of any ISurface.
//   * closestPoint()— Newton-Raphson closest-point projection onto any ISurface.

#include "gk/surface/ISurface.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace gk {

// ── Curvature from fundamental forms ─────────────────────────────────────────

/// Gaussian curvature at (u,v) computed from the surface's SurfacePoint.
inline double gaussianCurvature(const ISurface& surf, double u, double v)
{
    auto sp = surf.evaluate(u, v);
    Vec3 n  = sp.du.cross(sp.dv);
    double lenN = n.norm();
    if (lenN < 1e-14) return 0.0;
    n = n * (1.0 / lenN);

    double E = sp.du.dot(sp.du);
    double F = sp.du.dot(sp.dv);
    double G = sp.dv.dot(sp.dv);
    double L = sp.duu.dot(n);
    double M = sp.duv.dot(n);
    double N = sp.dvv.dot(n);

    double denom = E*G - F*F;
    return (std::abs(denom) > 0.0) ? (L*N - M*M) / denom : 0.0;
}

/// Mean curvature at (u,v) computed from the surface's SurfacePoint.
inline double meanCurvature(const ISurface& surf, double u, double v)
{
    auto sp = surf.evaluate(u, v);
    Vec3 n  = sp.du.cross(sp.dv);
    double lenN = n.norm();
    if (lenN < 1e-14) return 0.0;
    n = n * (1.0 / lenN);

    double E = sp.du.dot(sp.du);
    double F = sp.du.dot(sp.dv);
    double G = sp.dv.dot(sp.dv);
    double L = sp.duu.dot(n);
    double M = sp.duv.dot(n);
    double N = sp.dvv.dot(n);

    double denom = 2.0 * (E*G - F*F);
    return (std::abs(denom) > 0.0) ? (E*N - 2.0*F*M + G*L) / denom : 0.0;
}

// ── Triangle mesh ─────────────────────────────────────────────────────────────

/// Indexed triangle mesh produced by tessellate().
struct SurfaceMesh
{
    std::vector<Vec3>               vertices;   ///< World-space positions
    std::vector<Vec3>               normals;    ///< Per-vertex unit normals
    std::vector<std::array<int, 3>> triangles;  ///< Vertex index triples
};

// ── Tessellation ─────────────────────────────────────────────────────────────

/// Build a uniform (uSteps × vSteps) quad grid tessellated into triangles.
/// @param uSteps  Number of subdivisions along u (≥ 1).
/// @param vSteps  Number of subdivisions along v (≥ 1).
inline SurfaceMesh tessellate(const ISurface& surf, int uSteps, int vSteps)
{
    if (uSteps < 1 || vSteps < 1)
        throw std::invalid_argument("tessellate: steps must be >= 1");

    auto dom = surf.domain();

    // Clamp infinite domains to a finite working range.
    auto clampInterval = [](const Interval& iv, double lo, double hi) -> Interval {
        double a = (iv.lo < -1e9) ? lo : iv.lo;
        double b = (iv.hi >  1e9) ? hi : iv.hi;
        return Interval{a, b};
    };
    Interval iU = clampInterval(dom.u, 0.0, 1.0);
    Interval iV = clampInterval(dom.v, 0.0, 1.0);

    SurfaceMesh mesh;
    int rows = uSteps + 1, cols = vSteps + 1;
    mesh.vertices.reserve(rows * cols);
    mesh.normals .reserve(rows * cols);

    for (int i = 0; i < rows; ++i)
    {
        double u = iU.lo + (iU.hi - iU.lo) * (double(i) / double(uSteps));
        for (int j = 0; j < cols; ++j)
        {
            double v = iV.lo + (iV.hi - iV.lo) * (double(j) / double(vSteps));
            mesh.vertices.push_back(surf.evaluate(u, v).p);
            mesh.normals .push_back(surf.normalAt(u, v));
        }
    }

    mesh.triangles.reserve(2 * uSteps * vSteps);
    for (int i = 0; i < uSteps; ++i)
    {
        for (int j = 0; j < vSteps; ++j)
        {
            int i00 =  i      * cols + j;
            int i10 = (i + 1) * cols + j;
            int i01 =  i      * cols + (j + 1);
            int i11 = (i + 1) * cols + (j + 1);
            mesh.triangles.push_back({i00, i10, i11});
            mesh.triangles.push_back({i00, i11, i01});
        }
    }
    return mesh;
}

// ── Closest point (Newton-Raphson) ────────────────────────────────────────────

/// Project world point `pt` onto `surf` using Newton-Raphson iteration.
/// Starts from the initial guess (u0, v0) which must lie in the domain.
/// Returns (u, v) of the closest point found.
///
/// Convergence criterion: |f₁| < tol and |f₂| < tol where
///   f₁ = (S(u,v) - P)·S_u,   f₂ = (S(u,v) - P)·S_v.
inline std::pair<double,double>
closestPoint(const ISurface& surf, const Vec3& pt,
             double u0, double v0,
             int maxIter = 100, double tol = 1e-10)
{
    auto dom = surf.domain();
    double u = u0, v = v0;

    for (int iter = 0; iter < maxIter; ++iter)
    {
        auto sp = surf.evaluate(u, v);
        Vec3 diff = sp.p - pt;

        double f1 = diff.dot(sp.du);
        double f2 = diff.dot(sp.dv);

        if (std::abs(f1) <= tol && std::abs(f2) <= tol) break;

        // Jacobian of [f1, f2] w.r.t. [u, v]
        double j11 = sp.du.dot(sp.du) + diff.dot(sp.duu);
        double j12 = sp.du.dot(sp.dv) + diff.dot(sp.duv);
        double j22 = sp.dv.dot(sp.dv) + diff.dot(sp.dvv);

        double det = j11 * j22 - j12 * j12;
        if (std::abs(det) < 1e-15) break;    // near-singular: stop

        // Solve 2×2 system
        double du = (-f1 * j22 + f2 * j12) / det;
        double dv = (-f2 * j11 + f1 * j12) / det;

        u += du;
        v += dv;

        // Clamp to valid domain
        u = std::max(dom.u.lo, std::min(dom.u.hi, u));
        v = std::max(dom.v.lo, std::min(dom.v.hi, v));
    }
    return {u, v};
}

} // namespace gk
