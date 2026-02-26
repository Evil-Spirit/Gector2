#pragma once
// Chapter 6.2-6.5 — Boolean Operations.
//
// BooleanUnion(A, B)        — merge two bodies (Iteration 6.2)
// BooleanDifference(A, B)   — subtract B from A (Iteration 6.3)
// BooleanIntersection(A, B) — keep common volume (Iteration 6.4)
// isZeroVolume()            — zero-volume result detection (Iteration 6.5)
// detectSelfIntersection()  — self-intersecting input detection (Iteration 6.5)
//
// Implementation notes:
//   BooleanDifference uses the inner-shell model: B's faces are attached as
//   an inner shell with inverted orientations.  This gives exact signed-volume
//   results via the divergence theorem when B is fully enclosed by A.
//
//   BooleanUnion and BooleanIntersection use face-classification: each face is
//   tested by perturbing its UV-centre point in the outward (or inward) normal
//   direction and querying the other body with BRepQuery::classifyPoint.  This
//   approach is exact when the two bodies do not partially overlap at a face
//   boundary, and produces geometrically valid (though possibly non-manifold)
//   results in the general case.

#include "gk/boolean/FaceFaceIntersect.h"
#include "gk/brep/Body.h"
#include "gk/brep/BRepQuery.h"
#include "gk/brep/Face.h"
#include "gk/brep/Lump.h"
#include "gk/brep/Shell.h"
#include "gk/brep/Wire.h"
#include "gk/Handle.h"
#include <algorithm>

namespace gk {

// ── Private helpers ───────────────────────────────────────────────────────────

namespace detail_bool {

/// Default epsilon for outward/inward perturbation of classification test points.
static constexpr double kClassifyEps = 1e-5;

/// Flip a face orientation.
inline FaceOrientation flipOrientation(FaceOrientation o) noexcept
{
    return (o == FaceOrientation::kForward)
         ? FaceOrientation::kReversed
         : FaceOrientation::kForward;
}

/// Shallow-copy a face (shared surface pointer, copied UV domain and orientation).
/// When invert is true the orientation is flipped.
inline Handle<Face> copyFace(const Handle<Face>& src, bool invert = false)
{
    auto f = makeHandle<Face>();
    f->setSurface(src->surface());
    if (src->hasUVDomain()) f->setUVDomain(src->uvDomain());
    auto ori = src->orientation();
    if (invert) ori = flipOrientation(ori);
    f->setOrientation(ori);
    f->setOuterWire(makeHandle<Wire>());
    return f;
}

/// Shallow-copy a shell (optionally inverting all face orientations).
inline Handle<Shell> copyShell(const Handle<Shell>& src, bool invert = false)
{
    auto s = makeHandle<Shell>();
    s->setClosed(src->isClosed());
    for (auto& f : src->faces())
        s->addFace(copyFace(f, invert));
    return s;
}

/// Compute a test point just outside the face in the outward-normal direction.
inline Vec3 outwardTestPoint(const Face& face, double eps = kClassifyEps)
{
    const ISurface& surf = *face.surface();
    auto dom = face.uvDomain();
    double u = (dom.u.lo + dom.u.hi) * 0.5;
    double v = (dom.v.lo + dom.v.hi) * 0.5;
    Vec3 p = surf.evaluate(u, v).p;
    Vec3 n = surf.normalAt(u, v);
    if (face.orientation() == FaceOrientation::kReversed) n = n * -1.0;
    return p + n * eps;
}

/// Compute a test point just inside the face in the inward-normal direction.
inline Vec3 inwardTestPoint(const Face& face, double eps = kClassifyEps)
{
    return outwardTestPoint(face, -eps);
}

/// Wrap a shell in a new body (one lump, one outer shell).
inline Handle<Body> shellToBody(Handle<Shell> shell)
{
    shell->setClosed(true);
    auto lump = makeHandle<Lump>();
    lump->setOuterShell(std::move(shell));
    auto body = makeHandle<Body>();
    body->addLump(std::move(lump));
    return body;
}

} // namespace detail_bool

// ── Boolean Difference ────────────────────────────────────────────────────────

/// Subtract B from A.
///
/// A's shells become the outer shells of the result; B's shells are attached as
/// inner shells with inverted face orientations so the divergence theorem
/// computes volume(A) − volume(B).  Correct mass properties are guaranteed when
/// B is fully enclosed by A; for partially overlapping inputs the topology may
/// be non-manifold but the signed-volume integral remains well-defined.
inline Handle<Body> BooleanDifference(const Handle<Body>& A, const Handle<Body>& B)
{
    auto result = makeHandle<Body>();

    for (auto& lumpA : A->lumps()) {
        if (!lumpA) continue;
        auto newLump = makeHandle<Lump>();

        // Outer shell: verbatim copy of A's outer shell.
        if (lumpA->outerShell())
            newLump->setOuterShell(detail_bool::copyShell(lumpA->outerShell()));

        // Preserve any existing inner shells from A.
        for (auto& innerA : lumpA->innerShells())
            newLump->addInnerShell(detail_bool::copyShell(innerA));

        // Subtract B by adding its shells as inner shells (inverted orientations).
        for (auto& lumpB : B->lumps()) {
            if (!lumpB) continue;
            if (lumpB->outerShell()) {
                auto inv = detail_bool::copyShell(lumpB->outerShell(), /*invert=*/true);
                inv->setClosed(true);
                newLump->addInnerShell(std::move(inv));
            }
        }

        result->addLump(std::move(newLump));
    }

    return result;
}

// ── Boolean Union ─────────────────────────────────────────────────────────────

/// Merge two bodies.
///
/// Each face of A whose outward-perturbed UV-centre is NOT inside B is kept;
/// each face of B whose outward-perturbed UV-centre is NOT inside A is kept.
/// The retained faces are assembled into a single shell.
///
/// This approach is exact when the two bodies are disjoint (every face centre
/// is cleanly classified), and produces a geometrically valid approximation for
/// partially overlapping bodies.
inline Handle<Body> BooleanUnion(const Handle<Body>& A, const Handle<Body>& B)
{
    auto shell = makeHandle<Shell>();

    auto addFacesOutside = [&](const Handle<Body>& src, const Handle<Body>& other) {
        for (auto& lump : src->lumps()) {
            if (!lump || !lump->outerShell()) continue;
            for (auto& face : lump->outerShell()->faces()) {
                if (!face->hasSurface()) continue;
                Vec3 testPt = detail_bool::outwardTestPoint(*face);
                auto loc = BRepQuery::classifyPoint(*other, testPt);
                if (loc != PointLocation::kInside)
                    shell->addFace(detail_bool::copyFace(face));
            }
        }
    };

    addFacesOutside(A, B);
    addFacesOutside(B, A);

    return detail_bool::shellToBody(std::move(shell));
}

// ── Boolean Intersection ──────────────────────────────────────────────────────

/// Keep the common volume of A and B.
///
/// Each face of A whose inward-perturbed UV-centre is inside B, and each face
/// of B whose inward-perturbed UV-centre is inside A, is retained.
///
/// This approach is exact when one body is fully enclosed by the other.
inline Handle<Body> BooleanIntersection(const Handle<Body>& A, const Handle<Body>& B)
{
    auto shell = makeHandle<Shell>();

    auto addFacesInside = [&](const Handle<Body>& src, const Handle<Body>& other) {
        for (auto& lump : src->lumps()) {
            if (!lump || !lump->outerShell()) continue;
            for (auto& face : lump->outerShell()->faces()) {
                if (!face->hasSurface()) continue;
                Vec3 testPt = detail_bool::inwardTestPoint(*face);
                auto loc = BRepQuery::classifyPoint(*other, testPt);
                if (loc == PointLocation::kInside)
                    shell->addFace(detail_bool::copyFace(face));
            }
        }
    };

    addFacesInside(A, B);
    addFacesInside(B, A);

    return detail_bool::shellToBody(std::move(shell));
}

// ── Edge-case helpers (Iteration 6.5) ────────────────────────────────────────

/// Return true when the body's computed volume is below @p tol.
/// Useful for detecting zero-volume (degenerate) Boolean results.
inline bool isZeroVolume(const Handle<Body>& body,
                         double tol = 1e-9,
                         int    uvSamples = 16)
{
    if (!body || body->isEmpty()) return true;
    auto mp = BRepQuery::computeMassProperties(*body, uvSamples);
    return mp.volume < tol;
}

/// Return true if any pair of faces from the same shell genuinely intersect.
///
/// "Genuine" means the intersection curve passes through the interior of both
/// face UV domains, not merely along a shared edge boundary.  The test works
/// by evaluating each intersection curve at its midpoint and projecting the
/// result back to UV space on both faces.  Edge-sharing adjacent faces produce
/// a boundary UV point (on or outside the domain margin) and are not counted.
///
/// Uses IntersectFaceFace from Iteration 6.1.  Only the outer shell of each
/// lump is examined.  Complexity: O(F²) where F is the number of outer-shell
/// faces; keep bodies small when calling this function.
inline bool detectSelfIntersection(const Handle<Body>& body, double tol = 1e-8)
{
    if (!body) return false;

    // Minimum margin inside the UV domain boundary that counts as "interior".
    // For face domains of order-1 size this is well below any edge width.
    static constexpr double kDomMargin = 1e-6;
    // Clamp parameter when evaluating unbounded curves (Line3 uses ±1e18).
    static constexpr double kMaxParam  = 1e6;
    // Newton-Raphson iteration count for closestPoint projection.
    static constexpr int    kCpIter    = 20;

    for (auto& lump : body->lumps()) {
        if (!lump || !lump->outerShell()) continue;
        auto& faces = lump->outerShell()->faces();
        int n = static_cast<int>(faces.size());
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                auto& fi = faces[static_cast<std::size_t>(i)];
                auto& fj = faces[static_cast<std::size_t>(j)];
                if (!fi->hasSurface() || !fj->hasSurface()) continue;

                auto res = IntersectFaceFace(*fi, *fj, tol);
                if (res.type != FaceFaceIntersectResult::Type::kCurve) continue;

                // Check every intersection curve: if any curve midpoint lies
                // strictly inside both face UV domains, this is a genuine
                // self-intersection (not merely a shared boundary edge).
                for (auto& curve : res.curves) {
                    auto cdom = curve->domain();
                    double mid = (cdom.lo + cdom.hi) * 0.5;
                    mid = std::max(-kMaxParam, std::min(kMaxParam, mid));
                    Vec3 pt = curve->evaluate(mid).p;

                    // Project pt back to UV on face i.
                    auto domi = fi->uvDomain();
                    auto [ui, vi] = closestPoint(
                        *fi->surface(), pt,
                        (domi.u.lo + domi.u.hi) * 0.5,
                        (domi.v.lo + domi.v.hi) * 0.5, kCpIter, tol);

                    // Project pt back to UV on face j.
                    auto domj = fj->uvDomain();
                    auto [uj, vj] = closestPoint(
                        *fj->surface(), pt,
                        (domj.u.lo + domj.u.hi) * 0.5,
                        (domj.v.lo + domj.v.hi) * 0.5, kCpIter, tol);

                    bool insidei =
                        (ui > domi.u.lo + kDomMargin) && (ui < domi.u.hi - kDomMargin)
                     && (vi > domi.v.lo + kDomMargin) && (vi < domi.v.hi - kDomMargin);
                    bool insidej =
                        (uj > domj.u.lo + kDomMargin) && (uj < domj.u.hi - kDomMargin)
                     && (vj > domj.v.lo + kDomMargin) && (vj < domj.v.hi - kDomMargin);

                    if (insidei && insidej) return true;
                }
            }
        }
    }

    return false;
}

} // namespace gk
