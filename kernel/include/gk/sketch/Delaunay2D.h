#pragma once
// Chapter 5 — Bowyer-Watson Delaunay triangulation of a 2D point set.
// Supports a bounding outer polygon and optional hole polygons.
// Triangles whose centroids fall outside the outer polygon or inside a hole
// are removed from the result.

#include "gk/math/Vec2.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace gk {

struct DelauTri {
    int a, b, c;
};

struct DelaunayMesh2D {
    std::vector<Vec2>     vertices;   ///< original points only
    std::vector<DelauTri> triangles;
};

namespace detail_delaunay {

/// In-circumcircle test (Bowyer-Watson formulation).
inline bool inCircumcircle(const Vec2& A, const Vec2& B, const Vec2& C, const Vec2& D)
{
    double ax = A.x - D.x, ay = A.y - D.y;
    double bx = B.x - D.x, by = B.y - D.y;
    double cx = C.x - D.x, cy = C.y - D.y;
    return ax * (by * (cx*cx + cy*cy) - cy * (bx*bx + by*by))
         - ay * (bx * (cx*cx + cy*cy) - cx * (bx*bx + by*by))
         + (ax*ax + ay*ay) * (bx*cy - by*cx) > 0.0;
}

/// Ray-casting point-in-polygon test.
inline bool pointInPolygon(const Vec2& p, const std::vector<Vec2>& poly)
{
    int n = (int)poly.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& pi = poly[i];
        const Vec2& pj = poly[j];
        if (((pi.y > p.y) != (pj.y > p.y)) &&
            (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x)) {
            inside = !inside;
        }
    }
    return inside;
}

struct Triangle {
    int a, b, c;
};

struct Edge {
    int a, b;
    bool operator==(const Edge& o) const noexcept {
        return (a == o.a && b == o.b) || (a == o.b && b == o.a);
    }
};

} // namespace detail_delaunay

/// Bowyer-Watson Delaunay triangulation.
/// @param outerPoly  Boundary polygon (CCW winding expected).
/// @param holes      Hole polygons (CW winding, or just any polygon inside).
inline DelaunayMesh2D delaunayTriangulate(
    const std::vector<Vec2>& outerPoly,
    const std::vector<std::vector<Vec2>>& holes = {})
{
    using namespace detail_delaunay;

    // Collect all input points
    std::vector<Vec2> pts;
    pts.insert(pts.end(), outerPoly.begin(), outerPoly.end());
    for (auto& h : holes)
        pts.insert(pts.end(), h.begin(), h.end());

    int nOrig = (int)pts.size();
    if (nOrig < 3) return {};

    // Compute bounding box
    double minX = pts[0].x, maxX = pts[0].x;
    double minY = pts[0].y, maxY = pts[0].y;
    for (auto& p : pts) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    double dx = maxX - minX, dy = maxY - minY;
    double delta = std::max(dx, dy) * 2.0 + 1.0;

    // Super-triangle vertices (appended at the end)
    Vec2 st0{minX - delta,       minY - delta};
    Vec2 st1{minX + 2.0 * delta, minY - delta};
    Vec2 st2{minX + 0.5 * delta, minY + 2.0 * delta};
    int si0 = (int)pts.size(); pts.push_back(st0);
    int si1 = (int)pts.size(); pts.push_back(st1);
    int si2 = (int)pts.size(); pts.push_back(st2);

    std::vector<Triangle> tris;
    tris.push_back({si0, si1, si2});

    // Insert points one by one
    for (int pi = 0; pi < nOrig; ++pi) {
        const Vec2& p = pts[pi];

        // Find all triangles whose circumcircle contains p
        std::vector<Edge> boundary;
        std::vector<Triangle> keep;

        for (auto& tri : tris) {
            if (inCircumcircle(pts[tri.a], pts[tri.b], pts[tri.c], p)) {
                // Add edges to boundary
                boundary.push_back({tri.a, tri.b});
                boundary.push_back({tri.b, tri.c});
                boundary.push_back({tri.c, tri.a});
            } else {
                keep.push_back(tri);
            }
        }

        // Remove duplicate (shared) edges — keep only boundary edges
        std::vector<Edge> uniqueEdges;
        for (int i = 0; i < (int)boundary.size(); ++i) {
            bool dup = false;
            for (int j = 0; j < (int)boundary.size(); ++j) {
                if (i != j && boundary[i] == boundary[j]) { dup = true; break; }
            }
            if (!dup) uniqueEdges.push_back(boundary[i]);
        }

        // Re-triangulate the hole
        tris = keep;
        for (auto& e : uniqueEdges)
            tris.push_back({e.a, e.b, pi});
    }

    // Remove triangles sharing super-triangle vertices
    std::vector<Triangle> finalTris;
    for (auto& tri : tris) {
        if (tri.a == si0 || tri.a == si1 || tri.a == si2) continue;
        if (tri.b == si0 || tri.b == si1 || tri.b == si2) continue;
        if (tri.c == si0 || tri.c == si1 || tri.c == si2) continue;
        finalTris.push_back(tri);
    }

    // Filter: centroid must be inside outerPoly and not inside any hole
    DelaunayMesh2D result;
    result.vertices.assign(pts.begin(), pts.begin() + nOrig);

    for (auto& tri : finalTris) {
        Vec2 centroid{
            (pts[tri.a].x + pts[tri.b].x + pts[tri.c].x) / 3.0,
            (pts[tri.a].y + pts[tri.b].y + pts[tri.c].y) / 3.0
        };
        if (!pointInPolygon(centroid, outerPoly)) continue;
        bool inHole = false;
        for (auto& h : holes) {
            if (pointInPolygon(centroid, h)) { inHole = true; break; }
        }
        if (inHole) continue;
        result.triangles.push_back({tri.a, tri.b, tri.c});
    }

    return result;
}

} // namespace gk
