#pragma once
// Iteration 6.1 — Intersection Engine.
//
// IntersectFaceFace: compute intersection curves between two Face objects.
//
// Analytic specialisations:
//   Plane  ∩ Plane    → Line3  (or kCoincident / kNone)
//   Plane  ∩ Sphere   → Circle3 (or kPoint / kNone)
//   Sphere ∩ Sphere   → Circle3 (or kPoint / kNone)
//   Plane  ∩ Cylinder → Circle3 when the plane is perpendicular to the axis
//   Plane  ∩ Cone     → Circle3 when the plane is perpendicular to the axis
//
// All other surface pairs fall back to a robust surface–surface marching
// algorithm that returns a SampledCurve3 (piecewise-linear approximation).
//
// Corner-case handling:
//   - Tangent faces             → kPoint
//   - Coincident / parallel     → kCoincident or kNone
//   - Faces with no surface     → kNone

#include "gk/brep/Face.h"
#include "gk/curve/Circle.h"
#include "gk/curve/ICurve.h"
#include "gk/curve/Line.h"
#include "gk/surface/Cone.h"
#include "gk/surface/Cylinder.h"
#include "gk/surface/Plane.h"
#include "gk/surface/Sphere.h"
#include "gk/surface/SurfaceUtils.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace gk {

// ── SampledCurve3 ─────────────────────────────────────────────────────────────

/// Piecewise-linear 3-D curve with arc-length parameterisation.
/// Produced by the general surface–surface marching algorithm.
class SampledCurve3 : public ICurve3
{
public:
    /// Construct from an ordered sequence of world-space sample points.
    explicit SampledCurve3(std::vector<Vec3> pts) : pts_(std::move(pts))
    {
        params_.reserve(pts_.size());
        params_.push_back(0.0);
        for (std::size_t i = 1; i < pts_.size(); ++i)
            params_.push_back(params_.back() + (pts_[i] - pts_[i - 1]).norm());
    }

    CurvePoint3 evaluate(double t) const override
    {
        CurvePoint3 cp;
        cp.d2 = Vec3::zero();
        if (pts_.empty()) { cp.p = cp.d1 = Vec3::zero(); return cp; }
        if (pts_.size() == 1) { cp.p = pts_[0]; cp.d1 = Vec3::zero(); return cp; }

        t = std::max(0.0, std::min(params_.back(), t));

        // Binary search for the containing segment
        std::size_t lo = 0, hi = params_.size() - 1;
        while (hi - lo > 1) {
            std::size_t mid = (lo + hi) / 2;
            if (params_[mid] <= t) lo = mid; else hi = mid;
        }
        double seg   = params_[hi] - params_[lo];
        double alpha = (seg > 1e-15) ? (t - params_[lo]) / seg : 0.0;
        cp.p  = pts_[lo] * (1.0 - alpha) + pts_[hi] * alpha;
        cp.d1 = (seg > 1e-15) ? (pts_[hi] - pts_[lo]) : Vec3::zero();
        return cp;
    }

    Interval domain()   const override { return {0.0, params_.empty() ? 0.0 : params_.back()}; }
    bool     isClosed() const override
    {
        if (pts_.size() < 3) return false;
        return (pts_.front() - pts_.back()).norm() < 1e-6;
    }

    /// Direct access to the sample points (useful for tests and visualisation).
    const std::vector<Vec3>& points() const noexcept { return pts_; }

private:
    std::vector<Vec3>   pts_;
    std::vector<double> params_;
};

// ── FaceFaceIntersectResult ───────────────────────────────────────────────────

/// Result returned by IntersectFaceFace.
struct FaceFaceIntersectResult
{
    /// Classification of the geometric intersection.
    enum class Type {
        kNone,        ///< No intersection (surfaces miss, or faces have no surface).
        kPoint,       ///< Tangent contact: one or more isolated 3-D points.
        kCurve,       ///< One or more intersection curves.
        kCoincident   ///< Faces lie on the same (or coincident) surface.
    };

    Type                                  type   = Type::kNone;
    std::vector<std::shared_ptr<ICurve3>> curves;  ///< filled when type == kCurve
    std::vector<Vec3>                     points;  ///< filled when type == kPoint
};

// ── Forward declaration ───────────────────────────────────────────────────────

/// Compute the intersection between two faces.
///
/// @param tol  Geometric tolerance (default 1e-8).  Controls coincidence and
///             tangency classification as well as Newton-iteration convergence.
FaceFaceIntersectResult IntersectFaceFace(
    const Face& faceA,
    const Face& faceB,
    double tol = 1e-8);

// =============================================================================
// Implementation (header-only)
// =============================================================================

namespace detail_iff {

static constexpr double kPi = 3.14159265358979323846;

/// Domains wider than this (in parameter units) are treated as infinite
/// and clamped to a finite working range for seed searching.
static constexpr double kInfiniteDomainThreshold = 1e8;

/// Finite working range applied when an interval is flagged as infinite.
static constexpr double kFiniteDomainClamp = 50.0;

/// Loose multiplier used when comparing distances in seed-tolerance checks.
static constexpr double kSeedTolMultiplier = 1000.0;

/// Build a right-handed orthonormal basis {x, y} both perpendicular to unit n.
inline void buildBasis(const Vec3& n, Vec3& x, Vec3& y) noexcept
{
    Vec3 ref = (std::abs(n.x) < 0.9) ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 1.0, 0.0};
    x = (ref - n * n.dot(ref)).normalized();
    y = n.cross(x);
}

/// Return true when (u,v) lies inside dom extended by a margin on each side.
inline bool inDomain(double u, double v,
                     const SurfaceDomain& dom, double margin = 0.0) noexcept
{
    return u >= dom.u.lo - margin && u <= dom.u.hi + margin
        && v >= dom.v.lo - margin && v <= dom.v.hi + margin;
}

/// Clamp to domain.
inline void clampToDomain(double& u, double& v, const SurfaceDomain& dom) noexcept
{
    u = std::max(dom.u.lo, std::min(dom.u.hi, u));
    v = std::max(dom.v.lo, std::min(dom.v.hi, v));
}

/// Effective domain: clamp near-infinite intervals to a finite working range.
inline SurfaceDomain effectiveDomain(const SurfaceDomain& dom) noexcept
{
    auto clamp = [](const Interval& iv, double lo, double hi) -> Interval {
        return Interval{std::max(iv.lo, lo), std::min(iv.hi, hi)};
    };
    SurfaceDomain d;
    d.u = (dom.u.hi - dom.u.lo > kInfiniteDomainThreshold)
        ? clamp(dom.u, -kFiniteDomainClamp, kFiniteDomainClamp) : dom.u;
    d.v = (dom.v.hi - dom.v.lo > kInfiniteDomainThreshold)
        ? clamp(dom.v, -kFiniteDomainClamp, kFiniteDomainClamp) : dom.v;
    return d;
}

/// Alternating-projection Newton refinement.
/// Iteratively moves (u1,v1) on s1 and (u2,v2) on s2 toward a common point.
/// Returns true if convergence was reached within tolerance.
inline bool refinePair(
    const ISurface& s1, double& u1, double& v1,
    const ISurface& s2, double& u2, double& v2,
    double tol, int maxIter = 20)
{
    for (int k = 0; k < maxIter; ++k) {
        auto sp1 = s1.evaluate(u1, v1);
        auto sp2 = s2.evaluate(u2, v2);
        if ((sp1.p - sp2.p).squaredNorm() < tol * tol) return true;

        // Project sp2.p onto s1 starting from current (u1,v1)
        auto [nu1, nv1] = closestPoint(s1, sp2.p, u1, v1, 20, tol * 0.1);
        u1 = nu1; v1 = nv1;

        // Project updated s1 point onto s2
        sp1 = s1.evaluate(u1, v1);
        auto [nu2, nv2] = closestPoint(s2, sp1.p, u2, v2, 20, tol * 0.1);
        u2 = nu2; v2 = nv2;
    }
    auto sp1 = s1.evaluate(u1, v1);
    auto sp2 = s2.evaluate(u2, v2);
    return (sp1.p - sp2.p).squaredNorm() < tol * tol * 100.0;  // NOLINT
}

// ── Analytic: Plane ∩ Plane ───────────────────────────────────────────────────

inline FaceFaceIntersectResult planePlane(
    const Plane& pA, const Plane& pB, double tol)
{
    FaceFaceIntersectResult res;
    const Vec3 n1 = pA.normalAt(0, 0);
    const Vec3 n2 = pB.normalAt(0, 0);
    Vec3  lineDir  = n1.cross(n2);
    double sinAngle = lineDir.norm();

    if (sinAngle < tol) {
        // Parallel planes — coincident if offset is within tolerance
        double gap = std::abs((pB.origin() - pA.origin()).dot(n1));
        res.type = (gap < tol)
                 ? FaceFaceIntersectResult::Type::kCoincident
                 : FaceFaceIntersectResult::Type::kNone;
        return res;
    }

    lineDir = lineDir * (1.0 / sinAngle);   // normalise

    // Solve n1·p = c1, n2·p = c2, setting the axis-aligned component that
    // coincides with the largest component of lineDir to zero.
    const double c1 = n1.dot(pA.origin());
    const double c2 = n2.dot(pB.origin());

    auto solve2 = [](double a11, double a12, double b1,
                     double a21, double a22, double b2,
                     double& x,  double& y) -> bool {
        double det = a11 * a22 - a12 * a21;
        if (std::abs(det) < 1e-14) return false;
        x = (b1 * a22 - b2 * a12) / det;
        y = (a11 * b2 - a21 * b1) / det;
        return true;
    };

    Vec3 pt;
    bool ok = false;
    double ax = std::abs(lineDir.x), ay = std::abs(lineDir.y), az = std::abs(lineDir.z);
    if (ax >= ay && ax >= az) {
        double y, z;
        ok = solve2(n1.y, n1.z, c1, n2.y, n2.z, c2, y, z);
        pt = Vec3{0.0, y, z};
    } else if (ay >= az) {
        double x, z;
        ok = solve2(n1.x, n1.z, c1, n2.x, n2.z, c2, x, z);
        pt = Vec3{x, 0.0, z};
    } else {
        double x, y;
        ok = solve2(n1.x, n1.y, c1, n2.x, n2.y, c2, x, y);
        pt = Vec3{x, y, 0.0};
    }

    if (!ok) { res.type = FaceFaceIntersectResult::Type::kNone; return res; }

    res.type = FaceFaceIntersectResult::Type::kCurve;
    res.curves.push_back(std::make_shared<Line3>(Line3::unbounded(pt, lineDir)));
    return res;
}

// ── Analytic: Plane ∩ Sphere ─────────────────────────────────────────────────

inline FaceFaceIntersectResult planeSphere(
    const Plane& pl, const Sphere& sp, double tol)
{
    FaceFaceIntersectResult res;
    const Vec3 n           = pl.normalAt(0, 0);
    double     signedDist  = (sp.center() - pl.origin()).dot(n);
    double     r           = sp.radius();
    double     h2          = r * r - signedDist * signedDist;

    if (h2 < -tol * r) {
        res.type = FaceFaceIntersectResult::Type::kNone;
        return res;
    }

    Vec3 circCtr = sp.center() - n * signedDist;
    if (h2 < tol * r) {
        res.type = FaceFaceIntersectResult::Type::kPoint;
        res.points.push_back(circCtr);
        return res;
    }

    Vec3 x, y;
    buildBasis(n, x, y);
    res.type = FaceFaceIntersectResult::Type::kCurve;
    res.curves.push_back(std::make_shared<Circle3>(circCtr, std::sqrt(h2), x, y));
    return res;
}

// ── Analytic: Sphere ∩ Sphere ─────────────────────────────────────────────────

inline FaceFaceIntersectResult sphereSphere(
    const Sphere& s1, const Sphere& s2, double tol)
{
    FaceFaceIntersectResult res;
    Vec3   axis = s2.center() - s1.center();
    double d    = axis.norm();
    double r1 = s1.radius(), r2 = s2.radius();

    if (d < tol) {
        // Concentric
        res.type = (std::abs(r1 - r2) < tol)
                 ? FaceFaceIntersectResult::Type::kCoincident
                 : FaceFaceIntersectResult::Type::kNone;
        return res;
    }

    if (d > r1 + r2 + tol || d < std::abs(r1 - r2) - tol) {
        res.type = FaceFaceIntersectResult::Type::kNone;
        return res;
    }

    double a  = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
    double h2 = r1 * r1 - a * a;
    Vec3   n  = axis * (1.0 / d);

    if (h2 <= tol * r1 * 2.0) {
        res.type = FaceFaceIntersectResult::Type::kPoint;
        res.points.push_back(s1.center() + n * a);
        return res;
    }

    Vec3 x, y;
    buildBasis(n, x, y);
    res.type = FaceFaceIntersectResult::Type::kCurve;
    res.curves.push_back(
        std::make_shared<Circle3>(s1.center() + n * a, std::sqrt(h2), x, y));
    return res;
}

// ── Analytic: Plane ∩ Cylinder (axis-perpendicular case) ─────────────────────

/// Returns true and fills `res` when the plane is (nearly) perpendicular to
/// the cylinder axis, yielding a circle.  Otherwise returns false.
inline bool planeCylinderCircle(
    const Plane& pl, const Cylinder& cy,
    const SurfaceDomain& domCy,
    double tol,
    FaceFaceIntersectResult& res)
{
    const Vec3 n    = pl.normalAt(0, 0);
    const Vec3 ax   = cy.axis();
    double cosAngle = std::abs(n.dot(ax));

    if (cosAngle < 1.0 - tol) return false;   // not axis-perpendicular

    // Height v at which the plane intersects the cylinder axis
    double ndotax = n.dot(ax);
    if (std::abs(ndotax) < 1e-14) return false;
    double v0 = n.dot(pl.origin() - cy.origin()) / ndotax;

    if (v0 < domCy.v.lo - tol || v0 > domCy.v.hi + tol) {
        res.type = FaceFaceIntersectResult::Type::kNone;
        return true;
    }

    Vec3 circCenter = cy.origin() + ax * v0;
    Vec3 xAxis      = cy.uRef();
    Vec3 yAxis      = ax.cross(xAxis);

    res.type = FaceFaceIntersectResult::Type::kCurve;
    res.curves.push_back(
        std::make_shared<Circle3>(circCenter, cy.radius(), xAxis, yAxis));
    return true;
}

// ── Analytic: Plane ∩ Cone (axis-perpendicular case) ─────────────────────────

/// Returns true and fills `res` when the plane is (nearly) perpendicular to
/// the cone axis, yielding a circle.  Otherwise returns false.
inline bool planeConeCircle(
    const Plane& pl, const Cone& co,
    const SurfaceDomain& domCo,
    double tol,
    FaceFaceIntersectResult& res)
{
    const Vec3 n    = pl.normalAt(0, 0);
    const Vec3 ax   = co.axis();
    double cosAngle = std::abs(n.dot(ax));

    if (cosAngle < 1.0 - tol) return false;   // not axis-perpendicular

    double ndotax = n.dot(ax);
    if (std::abs(ndotax) < 1e-14) return false;
    double v0 = n.dot(pl.origin() - co.apex()) / ndotax;

    if (v0 < domCo.v.lo - tol || v0 > domCo.v.hi + tol) {
        res.type = FaceFaceIntersectResult::Type::kNone;
        return true;
    }

    double r = v0 * std::tan(co.halfAngle());
    if (r < tol) {
        res.type = FaceFaceIntersectResult::Type::kPoint;
        res.points.push_back(co.apex() + ax * v0);
        return true;
    }

    Vec3 circCenter = co.apex() + ax * v0;
    Vec3 xAxis      = co.uRef();
    Vec3 yAxis      = ax.cross(xAxis);

    res.type = FaceFaceIntersectResult::Type::kCurve;
    res.curves.push_back(
        std::make_shared<Circle3>(circCenter, r, xAxis, yAxis));
    return true;
}

// ── General: surface–surface marching ─────────────────────────────────────────

/// Estimate the approximate world-space scale of surface s over domain dom.
inline double surfaceScale(const ISurface& s, const SurfaceDomain& dom) noexcept
{
    double u = (dom.u.lo + dom.u.hi) * 0.5;
    double v = (dom.v.lo + dom.v.hi) * 0.5;
    auto sp = s.evaluate(u, v);
    double du = sp.du.norm() * (dom.u.hi - dom.u.lo);
    double dv = sp.dv.norm() * (dom.v.hi - dom.v.lo);
    return (du + dv) * 0.5;
}

/// March in direction `dir` (+1 or -1) along the surface-surface intersection.
/// `origin` is the starting world-space point, used to detect loop closure.
inline std::vector<Vec3> marchOne(
    const ISurface& s1, const SurfaceDomain& dom1,
    const ISurface& s2, const SurfaceDomain& dom2,
    double u1, double v1, double u2, double v2,
    double stepWorld, double dir, double tol, int maxSteps,
    const Vec3& origin)
{
    std::vector<Vec3> pts;
    auto sp1 = s1.evaluate(u1, v1);
    auto sp2 = s2.evaluate(u2, v2);
    pts.push_back((sp1.p + sp2.p) * 0.5);

    bool dom1Infinite = (dom1.u.hi - dom1.u.lo > 1e8) || (dom1.v.hi - dom1.v.lo > 1e8);
    bool dom2Infinite = (dom2.u.hi - dom2.u.lo > 1e8) || (dom2.v.hi - dom2.v.lo > 1e8);
    double margin = stepWorld * 2.0;

    for (int k = 0; k < maxSteps; ++k) {
        Vec3   n1    = s1.normalAt(u1, v1);
        Vec3   n2    = s2.normalAt(u2, v2);
        Vec3   tang  = n1.cross(n2);
        double tlen  = tang.norm();
        if (tlen < 1e-12) break;   // tangent contact: normals are parallel

        tang = tang * (dir / tlen);
        Vec3 pred = pts.back() + tang * stepWorld;

        // Project predicted point onto each surface
        auto [nu1, nv1] = closestPoint(s1, pred, u1, v1, 20, tol);
        auto [nu2, nv2] = closestPoint(s2, pred, u2, v2, 20, tol);
        u1 = nu1; v1 = nv1;
        u2 = nu2; v2 = nv2;

        // Newton refinement
        refinePair(s1, u1, v1, s2, u2, v2, tol);

        // Domain exit check: stop if we have left the finite face domain(s)
        bool in1 = dom1Infinite || inDomain(u1, v1, dom1, margin);
        bool in2 = dom2Infinite || inDomain(u2, v2, dom2, margin);
        if (!in1 || !in2) break;

        sp1 = s1.evaluate(u1, v1);
        sp2 = s2.evaluate(u2, v2);
        Vec3 newPt = (sp1.p + sp2.p) * 0.5;
        pts.push_back(newPt);

        // Closure check: have we looped back to the start?
        if (k > 4 && (newPt - origin).squaredNorm() < stepWorld * stepWorld * 4.0)
            break;
    }
    return pts;
}

/// General surface–surface marching intersection (fallback for non-analytic pairs).
inline FaceFaceIntersectResult marchingIntersect(
    const ISurface& s1, const SurfaceDomain& dom1,
    const ISurface& s2, const SurfaceDomain& dom2,
    double tol)
{
    FaceFaceIntersectResult res;

    // ── Effective (clamped) domains for seed searching ────────────────────────
    SurfaceDomain eff1 = effectiveDomain(dom1);
    SurfaceDomain eff2 = effectiveDomain(dom2);

    // ── Step size estimation ──────────────────────────────────────────────────
    double scale1 = surfaceScale(s1, eff1);
    double scale2 = surfaceScale(s2, eff2);
    double scale  = (std::min)(scale1 > 1e-9 ? scale1 : 1e20,
                               scale2 > 1e-9 ? scale2 : 1e20);
    if (scale > 1e18) scale = 1.0;
    double stepWorld = scale * 0.05;
    stepWorld = (std::max)(stepWorld, tol * kSeedTolMultiplier);
    stepWorld = (std::min)(stepWorld, 10.0);

    // ── Grid seed search ──────────────────────────────────────────────────────
    // Use a seed tolerance proportional to the geometric scale so that grid
    // points near (but not exactly on) the intersection are accepted as
    // candidates and later refined by Newton iteration.
    static constexpr int kGridN = 10;
    double seedTol   = (std::max)(scale * 0.4, tol * kSeedTolMultiplier);
    double seedTolSq = seedTol * seedTol;

    std::vector<std::array<double, 4>> seeds;
    // World-space positions of accepted seeds (for deduplication).
    std::vector<Vec3> seedPts;

    auto tryAddSeed = [&](double u1, double v1, double u2, double v2) {
        if (!inDomain(u1, v1, eff1, tol * kSeedTolMultiplier)) return;
        if (!inDomain(u2, v2, eff2, tol * kSeedTolMultiplier)) return;
        double ru1 = u1, rv1 = v1, ru2 = u2, rv2 = v2;
        if (!refinePair(s1, ru1, rv1, s2, ru2, rv2, tol)) return;
        // World-space deduplication: keep seeds at least stepWorld apart
        Vec3 pt = (s1.evaluate(ru1, rv1).p + s2.evaluate(ru2, rv2).p) * 0.5;
        double thSq = stepWorld * stepWorld;
        for (auto& sp : seedPts)
            if ((sp - pt).squaredNorm() < thSq) return;
        seeds.push_back({ru1, rv1, ru2, rv2});
        seedPts.push_back(pt);
    };

    // Helper: project world point `pt` onto `surf` trying several initial
    // guesses spread across the u-range.  Returns the (u,v) with the
    // smallest residual, avoiding saddle-point traps on periodic surfaces.
    auto bestProject = [&](const ISurface& surf, const SurfaceDomain& dom,
                           const Vec3& pt,
                           double u_hint, double v_hint) -> std::pair<double,double>
    {
        double bestDSq = 1e30;
        double bu = u_hint, bv = v_hint;
        static constexpr int kNGuess = 4;
        for (int ig = 0; ig <= kNGuess; ++ig) {
            double u0 = dom.u.lo + (dom.u.hi - dom.u.lo) * (double(ig) / kNGuess);
            auto [nu, nv] = closestPoint(surf, pt, u0, v_hint, 20, tol * 0.1);
            Vec3   pp = surf.evaluate(nu, nv).p;
            double d2 = (pp - pt).squaredNorm();
            if (d2 < bestDSq) { bestDSq = d2; bu = nu; bv = nv; }
        }
        return {bu, bv};
    };

    // Sample s1 domain, project each point onto s2
    double v2mid = (eff2.v.lo + eff2.v.hi) * 0.5;
    for (int i = 0; i <= kGridN; ++i) {
        double u1 = eff1.u.lo + (eff1.u.hi - eff1.u.lo) * (double(i) / kGridN);
        for (int j = 0; j <= kGridN; ++j) {
            double v1 = eff1.v.lo + (eff1.v.hi - eff1.v.lo) * (double(j) / kGridN);
            Vec3 p1 = s1.evaluate(u1, v1).p;
            auto [nu2, nv2] = bestProject(s2, eff2, p1, u1, v2mid);
            Vec3 p2 = s2.evaluate(nu2, nv2).p;
            if ((p1 - p2).squaredNorm() < seedTolSq)
                tryAddSeed(u1, v1, nu2, nv2);
        }
    }

    // Sample s2 domain, project each point onto s1 (catches cases when s1
    // has a much larger domain than s2, e.g. plane vs. cylinder)
    double v1mid = (eff1.v.lo + eff1.v.hi) * 0.5;
    for (int i = 0; i <= kGridN; ++i) {
        double u2 = eff2.u.lo + (eff2.u.hi - eff2.u.lo) * (double(i) / kGridN);
        for (int j = 0; j <= kGridN; ++j) {
            double v2 = eff2.v.lo + (eff2.v.hi - eff2.v.lo) * (double(j) / kGridN);
            Vec3 p2 = s2.evaluate(u2, v2).p;
            auto [nu1, nv1] = bestProject(s1, eff1, p2, u2, v1mid);
            Vec3 p1 = s1.evaluate(nu1, nv1).p;
            if ((p1 - p2).squaredNorm() < seedTolSq)
                tryAddSeed(nu1, nv1, u2, v2);
        }
    }

    if (seeds.empty()) {
        res.type = FaceFaceIntersectResult::Type::kNone;
        return res;
    }

    // ── Curve tracing ─────────────────────────────────────────────────────────
    std::vector<bool> seedUsed(seeds.size(), false);

    for (std::size_t si = 0; si < seeds.size(); ++si) {
        if (seedUsed[si]) continue;
        seedUsed[si] = true;

        auto& seed   = seeds[si];
        auto  sp1    = s1.evaluate(seed[0], seed[1]);
        auto  sp2    = s2.evaluate(seed[2], seed[3]);
        Vec3  origin = (sp1.p + sp2.p) * 0.5;

        auto fwd = marchOne(s1, dom1, s2, dom2,
                            seed[0], seed[1], seed[2], seed[3],
                            stepWorld, +1.0, tol, 500, origin);

        bool closed = fwd.size() > 4
                   && (fwd.back() - origin).norm() < stepWorld * 2.0;

        std::vector<Vec3> all;
        if (!closed) {
            auto bwd = marchOne(s1, dom1, s2, dom2,
                                seed[0], seed[1], seed[2], seed[3],
                                stepWorld, -1.0, tol, 500, origin);
            all.reserve(bwd.size() + fwd.size());
            for (int i = static_cast<int>(bwd.size()) - 1; i >= 1; --i)
                all.push_back(bwd[static_cast<std::size_t>(i)]);
            for (auto& p : fwd) all.push_back(p);
        } else {
            all = std::move(fwd);
        }

        if (all.size() >= 2) {
            res.type = FaceFaceIntersectResult::Type::kCurve;
            res.curves.push_back(std::make_shared<SampledCurve3>(all));

            // Mark any seed whose world-space position lies near this curve.
            double useTolSq = stepWorld * stepWorld * 16.0;
            for (std::size_t sj = si + 1; sj < seeds.size(); ++sj) {
                if (seedUsed[sj]) continue;
                Vec3 spt = seedPts[sj];
                for (auto& pt : all) {
                    if ((pt - spt).squaredNorm() < useTolSq) {
                        seedUsed[sj] = true;
                        break;
                    }
                }
            }
        }
    }

    if (res.curves.empty() && res.points.empty())
        res.type = FaceFaceIntersectResult::Type::kNone;

    return res;
}

} // namespace detail_iff

// ── IntersectFaceFace ─────────────────────────────────────────────────────────

inline FaceFaceIntersectResult IntersectFaceFace(
    const Face& faceA, const Face& faceB, double tol)
{
    if (!faceA.hasSurface() || !faceB.hasSurface())
        return {};

    const ISurface* sA = faceA.surface().get();
    const ISurface* sB = faceB.surface().get();
    SurfaceDomain   dA = faceA.uvDomain();
    SurfaceDomain   dB = faceB.uvDomain();

    // ── Analytic dispatch ─────────────────────────────────────────────────────
    const auto* plA = dynamic_cast<const Plane*>(sA);
    const auto* plB = dynamic_cast<const Plane*>(sB);
    const auto* spA = dynamic_cast<const Sphere*>(sA);
    const auto* spB = dynamic_cast<const Sphere*>(sB);
    const auto* cyA = dynamic_cast<const Cylinder*>(sA);
    const auto* cyB = dynamic_cast<const Cylinder*>(sB);
    const auto* coA = dynamic_cast<const Cone*>(sA);
    const auto* coB = dynamic_cast<const Cone*>(sB);

    if (plA && plB) return detail_iff::planePlane(*plA, *plB, tol);
    if (plA && spB) return detail_iff::planeSphere(*plA, *spB, tol);
    if (plB && spA) return detail_iff::planeSphere(*plB, *spA, tol);
    if (spA && spB) return detail_iff::sphereSphere(*spA, *spB, tol);

    if (plA && cyB) {
        FaceFaceIntersectResult r;
        if (detail_iff::planeCylinderCircle(*plA, *cyB, dB, tol, r)) return r;
    }
    if (plB && cyA) {
        FaceFaceIntersectResult r;
        if (detail_iff::planeCylinderCircle(*plB, *cyA, dA, tol, r)) return r;
    }

    if (plA && coB) {
        FaceFaceIntersectResult r;
        if (detail_iff::planeConeCircle(*plA, *coB, dB, tol, r)) return r;
    }
    if (plB && coA) {
        FaceFaceIntersectResult r;
        if (detail_iff::planeConeCircle(*plB, *coA, dA, tol, r)) return r;
    }

    // ── General marching fallback ─────────────────────────────────────────────
    return detail_iff::marchingIntersect(*sA, dA, *sB, dB, tol);
}

} // namespace gk
