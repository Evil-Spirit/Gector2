#pragma once
// Chapter 4.3 — BRepQuery: mass properties, point classification, topological distance.

#include "gk/brep/Body.h"
#include "gk/brep/Shell.h"
#include "gk/brep/Face.h"
#include <array>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <cstdint>

namespace gk {

struct MassProperties
{
    double surfaceArea{0.0};
    double volume{0.0};
    Vec3   centroid;
    double Ixx{0.0}, Iyy{0.0}, Izz{0.0};
    double Ixy{0.0}, Ixz{0.0}, Iyz{0.0};
};

enum class PointLocation { kInside, kOutside, kOnSurface };

class BRepQuery
{
public:
    // ── Mass properties ───────────────────────────────────────────────────────
    static MassProperties computeMassProperties(const Body& body, int uvSamples = 16)
    {
        MassProperties mp;
        double totalVol  = 0.0;
        Vec3   centAcc;

        for (auto& lump : body.lumps()) {
            if (!lump) continue;
            processShell(lump->outerShell(), mp, totalVol, centAcc, uvSamples);
            for (auto& s : lump->innerShells())
                processShell(s, mp, totalVol, centAcc, uvSamples);
        }

        mp.volume = std::abs(totalVol);
        if (mp.volume > 1e-30)
            mp.centroid = centAcc * (1.0 / totalVol);

        return mp;
    }

    // ── Point classification (ray casting +X) ────────────────────────────────
    static PointLocation classifyPoint(const Body& body,
                                       const Vec3& pt,
                                       double      tol = 1e-7)
    {
        // Check on-surface first
        for (auto& lump : body.lumps()) {
            if (!lump) continue;
            if (onSurface(lump->outerShell(), pt, tol)) return PointLocation::kOnSurface;
            for (auto& s : lump->innerShells())
                if (onSurface(s, pt, tol)) return PointLocation::kOnSurface;
        }

        // Ray-cast in +X direction
        int hits = 0;
        // Slightly perturb the ray off-axis to avoid degenerate intersections
        // at triangle vertices or edges shared by adjacent cells.
        const Vec3 dir = Vec3{1.0, 1e-5, 2e-5}.normalized();

        for (auto& lump : body.lumps()) {
            if (!lump) continue;
            hits += countIntersections(lump->outerShell(), pt, dir, tol);
            for (auto& s : lump->innerShells())
                hits += countIntersections(s, pt, dir, tol);
        }

        return (hits % 2 == 1) ? PointLocation::kInside : PointLocation::kOutside;
    }

    // ── Topological distance (BFS over shared edges) ─────────────────────────
    static int topologicalDistance(const Face&  a,
                                   const Face&  b,
                                   const Shell& shell)
    {
        const auto& faces = shell.faces();
        int n = static_cast<int>(faces.size());

        int idxA = -1, idxB = -1;
        for (int i = 0; i < n; ++i) {
            if (faces[i]->id() == a.id()) idxA = i;
            if (faces[i]->id() == b.id()) idxB = i;
        }
        if (idxA < 0 || idxB < 0) return -1;
        if (idxA == idxB) return 0;

        // Build edge → face-index map
        std::unordered_map<uint64_t, std::vector<int>> edgeToFaces;
        for (int i = 0; i < n; ++i) {
            if (!faces[i]->outerWire()) continue;
            for (auto& ce : faces[i]->outerWire()->coEdges()) {
                if (!ce || !ce->edge()) continue;
                edgeToFaces[ce->edge()->id().value()].push_back(i);
            }
        }

        // BFS
        std::vector<int> dist(static_cast<std::size_t>(n), -1);
        std::queue<int>  q;
        dist[static_cast<std::size_t>(idxA)] = 0;
        q.push(idxA);

        while (!q.empty()) {
            int cur = q.front(); q.pop();
            if (!faces[static_cast<std::size_t>(cur)]->outerWire()) continue;
            for (auto& ce : faces[static_cast<std::size_t>(cur)]->outerWire()->coEdges()) {
                if (!ce || !ce->edge()) continue;
                auto it = edgeToFaces.find(ce->edge()->id().value());
                if (it == edgeToFaces.end()) continue;
                for (int next : it->second) {
                    if (dist[static_cast<std::size_t>(next)] < 0) {
                        dist[static_cast<std::size_t>(next)] = dist[static_cast<std::size_t>(cur)] + 1;
                        if (next == idxB) return dist[static_cast<std::size_t>(next)];
                        q.push(next);
                    }
                }
            }
        }
        return -1;
    }

private:
    using Tri = std::array<Vec3, 3>;

    // ── Tessellate a face into triangles ─────────────────────────────────────
    static std::vector<Tri> tessellate(const Handle<Face>& face, int uvSamples)
    {
        std::vector<Tri> tris;
        if (!face->hasSurface() || !face->hasUVDomain()) return tris;

        auto&  surf  = face->surface();
        auto   dom   = face->uvDomain();
        double uStep = dom.u.width() / double(uvSamples);
        double vStep = dom.v.width() / double(uvSamples);

        for (int i = 0; i < uvSamples; ++i) {
            for (int j = 0; j < uvSamples; ++j) {
                double u0 = dom.u.lo + double(i)   * uStep;
                double u1 = dom.u.lo + double(i+1) * uStep;
                double v0 = dom.v.lo + double(j)   * vStep;
                double v1 = dom.v.lo + double(j+1) * vStep;

                Vec3 p00 = surf->evaluate(u0, v0).p;
                Vec3 p10 = surf->evaluate(u1, v0).p;
                Vec3 p01 = surf->evaluate(u0, v1).p;
                Vec3 p11 = surf->evaluate(u1, v1).p;

                tris.push_back({p00, p10, p11});
                tris.push_back({p00, p11, p01});
            }
        }
        return tris;
    }

    // ── Process one shell for mass-property accumulation ─────────────────────
    static void processShell(const Handle<Shell>& shell,
                              MassProperties&      mp,
                              double&              totalVol,
                              Vec3&                centAcc,
                              int                  uvSamples)
    {
        if (!shell) return;
        for (auto& face : shell->faces()) {
            if (!face || !face->hasSurface() || !face->hasUVDomain()) continue;

            double sign = (face->orientation() == FaceOrientation::kForward) ? 1.0 : -1.0;
            auto   tris = tessellate(face, uvSamples);

            for (auto& tri : tris) {
                const Vec3& p0 = tri[0];
                const Vec3& p1 = tri[1];
                const Vec3& p2 = tri[2];

                Vec3   e1   = p1 - p0;
                Vec3   e2   = p2 - p0;
                Vec3   n    = e1.cross(e2);
                double area = n.norm() * 0.5;
                mp.surfaceArea += area;

                // Signed tet volume via divergence theorem
                double sv = sign * p0.dot(p1.cross(p2)) / 6.0;
                totalVol  += sv;

                // Centroid accumulation (weighted by signed tet volume)
                Vec3 triCentroid = (p0 + p1 + p2) * (1.0 / 3.0);
                centAcc += triCentroid * sv;
            }
        }
    }

    // ── On-surface test (within tolerance of any triangle) ───────────────────
    static bool onSurface(const Handle<Shell>& shell, const Vec3& pt, double tol)
    {
        if (!shell) return false;
        for (auto& face : shell->faces()) {
            if (!face || !face->hasSurface() || !face->hasUVDomain()) continue;
            for (auto& tri : tessellate(face, 16)) {
                if (pointOnTriangle(pt, tri[0], tri[1], tri[2], tol))
                    return true;
            }
        }
        return false;
    }

    // ── Count +X ray intersections with a shell ──────────────────────────────
    static int countIntersections(const Handle<Shell>& shell,
                                   const Vec3&          orig,
                                   const Vec3&          dir,
                                   double               tol)
    {
        if (!shell) return 0;
        int hits = 0;
        for (auto& face : shell->faces()) {
            if (!face || !face->hasSurface() || !face->hasUVDomain()) continue;
            for (auto& tri : tessellate(face, 16)) {
                if (rayIntersectsTriangle(orig, dir, tri[0], tri[1], tri[2], tol))
                    ++hits;
            }
        }
        return hits;
    }

    // ── Geometry helpers ──────────────────────────────────────────────────────
    static bool pointOnTriangle(const Vec3& p,
                                 const Vec3& a, const Vec3& b, const Vec3& c,
                                 double tol)
    {
        Vec3   n  = (b - a).cross(c - a);
        double nl = n.norm();
        if (nl < tol) return false;
        if (std::abs((p - a).dot(n) / nl) > tol) return false;

        // Barycentric
        Vec3   v0 = c - a, v1 = b - a, v2 = p - a;
        double d00 = v0.dot(v0), d01 = v0.dot(v1), d02 = v0.dot(v2);
        double d11 = v1.dot(v1), d12 = v1.dot(v2);
        double denom = d00 * d11 - d01 * d01;
        if (std::abs(denom) < tol * tol) return false;
        double inv = 1.0 / denom;
        double u   = (d11 * d02 - d01 * d12) * inv;
        double v   = (d00 * d12 - d01 * d02) * inv;
        return (u >= -tol) && (v >= -tol) && (u + v <= 1.0 + tol);
    }

    /// Möller–Trumbore ray–triangle intersection; returns true if t > tol.
    static bool rayIntersectsTriangle(const Vec3& orig, const Vec3& dir,
                                       const Vec3& a,    const Vec3& b, const Vec3& c,
                                       double tol)
    {
        Vec3   e1  = b - a;
        Vec3   e2  = c - a;
        Vec3   h   = dir.cross(e2);
        double det = e1.dot(h);
        if (std::abs(det) < tol) return false;
        double invDet = 1.0 / det;
        Vec3   s      = orig - a;
        double u      = s.dot(h) * invDet;
        if (u < 0.0 || u > 1.0) return false;
        Vec3   q = s.cross(e1);
        double v = dir.dot(q) * invDet;
        if (v < 0.0 || u + v > 1.0) return false;
        double t = e2.dot(q) * invDet;
        return t > tol;
    }
};

} // namespace gk
