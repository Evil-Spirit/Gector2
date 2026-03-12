#pragma once
// Chapter 5 — More primitive builders: Torus, Wedge mesh, Prism mesh, Pyramid mesh.

#include "gk/brep/Body.h"
#include "gk/brep/Lump.h"
#include "gk/brep/Shell.h"
#include "gk/brep/Face.h"
#include "gk/brep/Wire.h"
#include "gk/Handle.h"
#include "gk/surface/Torus.h"
#include "gk/surface/DiscSurface.h"
#include "gk/math/Vec3.h"
#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace gk {

// ─────────────────────────────────────────────────────────────────────────────
// makeTorus
// ─────────────────────────────────────────────────────────────────────────────

inline Handle<Body> makeTorus(Vec3 center, Vec3 axis, double majorR, double minorR)
{
    static constexpr double kPi = 3.14159265358979323846;

    auto face = makeHandle<Face>();
    face->setSurface(std::make_shared<Torus>(center, axis, majorR, minorR));
    face->setUVDomain(SurfaceDomain{Interval{0.0, 2.0*kPi}, Interval{0.0, 2.0*kPi}});
    face->setOrientation(FaceOrientation::kForward);
    face->setOuterWire(makeHandle<Wire>());

    auto shell = makeHandle<Shell>();
    shell->addFace(std::move(face));
    shell->setClosed(true);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(std::move(shell));
    auto body = makeHandle<Body>();
    body->addLump(std::move(lump));
    return body;
}

// ─────────────────────────────────────────────────────────────────────────────
// SimpleMesh3D — polygon mesh (not B-Rep)
// ─────────────────────────────────────────────────────────────────────────────

struct SimpleMesh3D {
    std::vector<Vec3>               vertices;
    std::vector<Vec3>               normals;
    std::vector<std::array<int, 3>> triangles;
};

// ─────────────────────────────────────────────────────────────────────────────
// makeWedgeMesh
// ─────────────────────────────────────────────────────────────────────────────
//
// Wedge: 6 vertices (triangular prism shape), 8 triangles.
// v0..v3 = bottom quad (z = corner.z), but only v0,v1,v3(=v3) form the base.
// Actually: triangular prism with 2 triangular ends (left/right) and 3 rect sides.
//
//  v0=(x,   y,   z),   v1=(x+dx, y,   z)   <- front-bottom edge
//  v2=(x+dx, y+dy, z), v3=(x,   y+dy, z)   <- back-bottom edge
//  v4=(x,   y+dy, z+dz), v5=(x+dx, y+dy, z+dz) <- top-back edge
//
// Faces:
//   Bottom quad (v0,v1,v2,v3) normal = -Z
//   Back rect  (v3,v2,v5,v4) normal = +Y
//   Front tri  (v0,v1,v2) ... wait, that's not triangular
//
// Let me redefine for a proper wedge (triangular prism):
//   Triangular cross-section in XZ plane, extruded along Y.
//   Bottom tri: v0=(x,y,z), v1=(x+dx,y,z), v2=(x,y,z+dz)
//   Top tri:    v3=(x,y+dy,z), v4=(x+dx,y+dy,z), v5=(x,y+dy,z+dz)
//
// Per spec:
//   v0=(corner), v1=(corner.x+dx,corner.y,corner.z),
//   v2=(corner.x+dx,corner.y+dy,corner.z), v3=(corner.x,corner.y+dy,corner.z)
//   v4=(corner.x,corner.y+dy,corner.z+dz), v5=(corner.x+dx,corner.y+dy,corner.z+dz)
//
// Spec says: bottom(quad), back(quad), front(tri), top(quad), leftTri, rightTri
// So the cross-section is a right triangle in the YZ plane.

inline SimpleMesh3D makeWedgeMesh(Vec3 corner, double dx, double dy, double dz)
{
    double x = corner.x, y = corner.y, z = corner.z;

    // Per spec vertex layout
    Vec3 v0{x,      y,      z     };
    Vec3 v1{x + dx, y,      z     };
    Vec3 v2{x + dx, y + dy, z     };
    Vec3 v3{x,      y + dy, z     };
    Vec3 v4{x,      y + dy, z + dz};
    Vec3 v5{x + dx, y + dy, z + dz};

    SimpleMesh3D mesh;
    mesh.vertices = {v0, v1, v2, v3, v4, v5};

    // Helper lambda to compute flat normal
    auto triNorm = [](const Vec3& a, const Vec3& b, const Vec3& c) -> Vec3 {
        Vec3 n = (b - a).cross(c - a);
        double len = n.norm();
        return (len > 1e-14) ? n * (1.0 / len) : Vec3::zero();
    };

    // Bottom quad (v0,v3,v2,v1) — faces down (-Z)
    // Two triangles: (v0,v3,v2) and (v0,v2,v1)
    Vec3 nBottom = triNorm(v0, v3, v2); // should be approx -Z
    mesh.normals.push_back(nBottom); int nBot = 0;
    mesh.triangles.push_back({0, 3, 2});
    mesh.triangles.push_back({0, 2, 1});

    // Back quad (v3,v4,v5,v2) — faces +Y
    Vec3 nBack = triNorm(v3, v4, v5);
    mesh.normals.push_back(nBack); int nBk = 1;
    mesh.triangles.push_back({3, 4, 5});
    mesh.triangles.push_back({3, 5, 2});

    // Top quad (v4,v3,v2,v5) ... wait spec says "top(quad)" not "top(tri)"
    // The top is the slanted face: v0,v1,v5,v4 (the slant from bottom-front to top-back)
    Vec3 nTop = triNorm(v0, v1, v5);
    mesh.normals.push_back(nTop); int nTp = 2;
    mesh.triangles.push_back({0, 1, 5});
    mesh.triangles.push_back({0, 5, 4});

    // Left tri (v0,v3,v4) — faces -X
    Vec3 nLeft = triNorm(v0, v3, v4);
    mesh.normals.push_back(nLeft); int nLt = 3;
    mesh.triangles.push_back({0, 3, 4});

    // Right tri (v1,v2,v5) — faces +X
    Vec3 nRight = triNorm(v1, v2, v5);
    mesh.normals.push_back(nRight); int nRt = 4;
    mesh.triangles.push_back({1, 2, 5});

    // Front tri (v0,v1,?) — the front is just v0,v1 edge at y=corner.y, no face
    // Actually per spec there's a front tri. Front is at y = corner.y: v0, v1 and... 
    // the wedge has no vertex at front-top so it's a degenerate edge. 
    // The "front" face is v0,v1 which collapses to an edge — there's no front face.
    // The actual shape described by v0..v5 is a triangular prism where:
    // - Two triangular ends: left (v0,v3,v4) and right (v1,v2,v5)  
    // - Three rectangular faces: bottom (v0,v1,v2,v3), back (v3,v2,v5,v4), top-slant (v0,v1,v5,v4)

    (void)nBot; (void)nBk; (void)nTp; (void)nLt; (void)nRt;

    // Re-assign per-triangle normals (flat shading: each triangle gets one normal index)
    // We need to expand normals so each triangle references its own normal.
    // Rebuild with per-triangle normals:
    mesh.normals.clear();
    mesh.triangles.clear();

    struct Tri { int a, b, c; Vec3 n; };
    std::vector<Tri> tris = {
        // Bottom quad → 2 tris, normal -Z ish
        {0, 3, 2, triNorm(v0, v3, v2)},
        {0, 2, 1, triNorm(v0, v2, v1)},
        // Back quad → 2 tris
        {3, 4, 5, triNorm(v3, v4, v5)},
        {3, 5, 2, triNorm(v3, v5, v2)},
        // Top-slant quad → 2 tris
        {0, 1, 5, triNorm(v0, v1, v5)},
        {0, 5, 4, triNorm(v0, v5, v4)},
        // Left tri
        {0, 4, 3, triNorm(v0, v4, v3)},
        // Right tri
        {1, 2, 5, triNorm(v1, v2, v5)},
    };

    for (auto& t : tris) {
        mesh.normals.push_back(t.n);
        mesh.triangles.push_back({t.a, t.b, t.c});
    }
    // Note: normals[i] corresponds to triangles[i] (flat shading)
    // For export, caller uses normals[triangle_index] not normals[vertex_index]
    // But ObjWriter uses per-vertex normals. We'll duplicate vertices per triangle.

    // Rebuild with per-triangle vertices (flat normals)
    mesh.vertices.clear();
    mesh.normals.clear();
    mesh.triangles.clear();
    const Vec3 verts[6] = {v0, v1, v2, v3, v4, v5};
    for (auto& t : tris) {
        int base = (int)mesh.vertices.size();
        mesh.vertices.push_back(verts[t.a]);
        mesh.vertices.push_back(verts[t.b]);
        mesh.vertices.push_back(verts[t.c]);
        mesh.normals.push_back(t.n);
        mesh.normals.push_back(t.n);
        mesh.normals.push_back(t.n);
        mesh.triangles.push_back({base, base+1, base+2});
    }

    return mesh;
}

// ─────────────────────────────────────────────────────────────────────────────
// makePrismMesh
// ─────────────────────────────────────────────────────────────────────────────

inline SimpleMesh3D makePrismMesh(Vec3 baseCenter, Vec3 axis, double radius,
                                   int nSides, double height)
{
    static constexpr double kPi = 3.14159265358979323846;
    Vec3 axN = axis.normalized();
    Vec3 uRef, vRef;
    DiscSurface::buildFrame(axN, uRef, vRef);

    // Build base and top polygon vertices
    std::vector<Vec3> baseVerts(nSides), topVerts(nSides);
    for (int i = 0; i < nSides; ++i) {
        double angle = 2.0 * kPi * double(i) / double(nSides);
        Vec3 radial = uRef * std::cos(angle) + vRef * std::sin(angle);
        baseVerts[i] = baseCenter + radial * radius;
        topVerts[i]  = baseVerts[i] + axN * height;
    }

    SimpleMesh3D mesh;
    auto addTri = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 n) {
        int base = (int)mesh.vertices.size();
        mesh.vertices.push_back(a);
        mesh.vertices.push_back(b);
        mesh.vertices.push_back(c);
        mesh.normals.push_back(n);
        mesh.normals.push_back(n);
        mesh.normals.push_back(n);
        mesh.triangles.push_back({base, base+1, base+2});
    };

    // Base cap (fan triangulation, normal = -axN)
    Vec3 nBase = axN * -1.0;
    for (int i = 0; i < nSides; ++i) {
        int j = (i + 1) % nSides;
        addTri(baseCenter, baseVerts[j], baseVerts[i], nBase);
    }

    // Top cap (fan triangulation, normal = +axN)
    Vec3 topCenter = baseCenter + axN * height;
    Vec3 nTop = axN;
    for (int i = 0; i < nSides; ++i) {
        int j = (i + 1) % nSides;
        addTri(topCenter, topVerts[i], topVerts[j], nTop);
    }

    // Side quads (split into 2 triangles each)
    for (int i = 0; i < nSides; ++i) {
        int j = (i + 1) % nSides;
        Vec3 b0 = baseVerts[i], b1 = baseVerts[j];
        Vec3 t0 = topVerts[i],  t1 = topVerts[j];
        // Outward radial normal (average of edge midpoint)
        double ai = 2.0 * kPi * (double(i) + 0.5) / double(nSides);
        Vec3 nSide = (uRef * std::cos(ai) + vRef * std::sin(ai));
        addTri(b0, b1, t1, nSide);
        addTri(b0, t1, t0, nSide);
    }

    return mesh;
}

// ─────────────────────────────────────────────────────────────────────────────
// makePyramidMesh
// ─────────────────────────────────────────────────────────────────────────────

inline SimpleMesh3D makePyramidMesh(Vec3 apex, Vec3 baseCenter, Vec3 axisUp,
                                     double radius, int nSides)
{
    static constexpr double kPi = 3.14159265358979323846;
    Vec3 axN = axisUp.normalized();
    Vec3 uRef, vRef;
    DiscSurface::buildFrame(axN, uRef, vRef);

    // Build base polygon vertices
    std::vector<Vec3> baseVerts(nSides);
    for (int i = 0; i < nSides; ++i) {
        double angle = 2.0 * kPi * double(i) / double(nSides);
        Vec3 radial = uRef * std::cos(angle) + vRef * std::sin(angle);
        baseVerts[i] = baseCenter + radial * radius;
    }

    SimpleMesh3D mesh;
    auto addTri = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 n) {
        int base = (int)mesh.vertices.size();
        mesh.vertices.push_back(a);
        mesh.vertices.push_back(b);
        mesh.vertices.push_back(c);
        mesh.normals.push_back(n);
        mesh.normals.push_back(n);
        mesh.normals.push_back(n);
        mesh.triangles.push_back({base, base+1, base+2});
    };

    // Base cap (fan, normal = -axN)
    Vec3 nBase = axN * -1.0;
    for (int i = 0; i < nSides; ++i) {
        int j = (i + 1) % nSides;
        addTri(baseCenter, baseVerts[j], baseVerts[i], nBase);
    }

    // Side triangles
    for (int i = 0; i < nSides; ++i) {
        int j = (i + 1) % nSides;
        Vec3 b0 = baseVerts[i], b1 = baseVerts[j];
        Vec3 n = (b1 - b0).cross(apex - b0);
        double len = n.norm();
        if (len > 1e-14) n = n * (1.0 / len);
        addTri(b0, b1, apex, n);
    }

    return mesh;
}

// ─────────────────────────────────────────────────────────────────────────────
// meshVolume
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the volume of a closed triangulated SimpleMesh3D using the
/// divergence theorem: V = (1/6) |Σ_triangles v0 · (v1 × v2)|.
/// Requires a consistently-wound, watertight mesh.
inline double meshVolume(const SimpleMesh3D& mesh)
{
    double vol = 0.0;
    for (auto& t : mesh.triangles) {
        const Vec3& v0 = mesh.vertices[static_cast<std::size_t>(t[0])];
        const Vec3& v1 = mesh.vertices[static_cast<std::size_t>(t[1])];
        const Vec3& v2 = mesh.vertices[static_cast<std::size_t>(t[2])];
        vol += v0.dot(v1.cross(v2));
    }
    return std::abs(vol) / 6.0;
}

} // namespace gk
