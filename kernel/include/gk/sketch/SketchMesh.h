#pragma once
// Chapter 5 — Sketch mesh: triangulate a 2D sketch region and lift to 3D.
// Also provides extrusion and revolution mesh generation.

#include "gk/sketch/SketchRegion.h"
#include "gk/sketch/Delaunay2D.h"
#include "gk/math/Vec3.h"
#include <array>
#include <cmath>
#include <vector>

namespace gk {

// ─────────────────────────────────────────────────────────────────────────────
// SketchMesh3D — flat 3D mesh from a 2D triangulation
// ─────────────────────────────────────────────────────────────────────────────

struct SketchMesh3D {
    std::vector<Vec3>               vertices;
    std::vector<Vec3>               normals;
    std::vector<std::array<int, 3>> triangles;
};

// ─────────────────────────────────────────────────────────────────────────────
// ExtrusionMesh3D — caps + side walls
// ─────────────────────────────────────────────────────────────────────────────

struct ExtrusionMesh3D {
    std::vector<Vec3>               vertices;
    std::vector<Vec3>               normals;
    std::vector<std::array<int, 3>> triangles;
    double                          volume{0.0};
};

// ─────────────────────────────────────────────────────────────────────────────
// triangulateSketch
// ─────────────────────────────────────────────────────────────────────────────

/// Triangulate a flat sketch region and lift it to 3D.
/// @param planeOrigin  Origin of the plane in 3D.
/// @param uAxis        First in-plane direction (maps to x in 2D).
/// @param vAxis        Second in-plane direction (maps to y in 2D).
inline SketchMesh3D triangulateSketch(const SketchRegion& region,
                                       Vec3 planeOrigin = Vec3::zero(),
                                       Vec3 uAxis = Vec3::unitX(),
                                       Vec3 vAxis = Vec3::unitY(),
                                       int  samplesPerCurve = 32)
{
    // Discretize loops
    auto outerPoly = region.discretizeOuter(samplesPerCurve);
    std::vector<std::vector<Vec2>> holePoly;
    holePoly.reserve(region.holes.size());
    for (int i = 0; i < (int)region.holes.size(); ++i)
        holePoly.push_back(region.discretizeHole(i, samplesPerCurve));

    // Triangulate
    auto mesh2d = delaunayTriangulate(outerPoly, holePoly);

    // Lift to 3D
    Vec3 normal = uAxis.cross(vAxis).normalized();

    SketchMesh3D result;
    result.vertices.reserve(mesh2d.vertices.size());
    result.normals.reserve(mesh2d.vertices.size());
    for (auto& p : mesh2d.vertices) {
        result.vertices.push_back(planeOrigin + uAxis * p.x + vAxis * p.y);
        result.normals.push_back(normal);
    }

    result.triangles.reserve(mesh2d.triangles.size());
    for (auto& t : mesh2d.triangles)
        result.triangles.push_back({t.a, t.b, t.c});

    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: shoelace area of a 2D polygon
// ─────────────────────────────────────────────────────────────────────────────

namespace detail_sketch {
inline double shoelaceArea(const std::vector<Vec2>& poly)
{
    double area = 0.0;
    int n = (int)poly.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        area += poly[j].x * poly[i].y;
        area -= poly[i].x * poly[j].y;
    }
    return std::abs(area) * 0.5;
}
} // namespace detail_sketch

// ─────────────────────────────────────────────────────────────────────────────
// extrudeSketch
// ─────────────────────────────────────────────────────────────────────────────

inline ExtrusionMesh3D extrudeSketch(const SketchRegion& region,
                                      Vec3 extrudeDir,
                                      int  samplesPerCurve = 32,
                                      int  extrudeSteps    = 1)
{
    auto outerPoly = region.discretizeOuter(samplesPerCurve);
    std::vector<std::vector<Vec2>> holePoly;
    for (int i = 0; i < (int)region.holes.size(); ++i)
        holePoly.push_back(region.discretizeHole(i, samplesPerCurve));

    double extLen = extrudeDir.norm();
    Vec3   extN   = (extLen > 1e-14) ? extrudeDir * (1.0 / extLen) : Vec3::unitZ();

    // Build a local coordinate frame for the sketch plane
    // Use two vectors perpendicular to extrudeDir as uAxis and vAxis
    Vec3 uAxis, vAxis;
    {
        Vec3 ref = (std::abs(extN.dot(Vec3::unitX())) < 0.9)
                   ? Vec3::unitX() : Vec3::unitY();
        uAxis = (ref - extN * ref.dot(extN)).normalized();
        vAxis = extN.cross(uAxis);
    }

    double area = detail_sketch::shoelaceArea(outerPoly);

    ExtrusionMesh3D result;
    result.volume = area * extLen;

    // Helper: add a triangle with flat normal
    auto addTri = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 n) {
        int base = (int)result.vertices.size();
        result.vertices.push_back(a);
        result.vertices.push_back(b);
        result.vertices.push_back(c);
        result.normals.push_back(n);
        result.normals.push_back(n);
        result.normals.push_back(n);
        result.triangles.push_back({base, base+1, base+2});
    };

    // Bottom cap (at planeOrigin=zero, normal = -extN)
    {
        auto mesh2d = delaunayTriangulate(outerPoly, holePoly);
        Vec3 capNorm = extN * -1.0;
        std::vector<Vec3> v3d;
        v3d.reserve(mesh2d.vertices.size());
        for (auto& p : mesh2d.vertices)
            v3d.push_back(uAxis * p.x + vAxis * p.y);
        // Bottom cap: reversed winding for outward normal (-extN)
        for (auto& t : mesh2d.triangles)
            addTri(v3d[t.a], v3d[t.c], v3d[t.b], capNorm);
    }

    // Top cap (at planeOrigin + extrudeDir, normal = +extN)
    {
        auto mesh2d = delaunayTriangulate(outerPoly, holePoly);
        Vec3 capNorm = extN;
        std::vector<Vec3> v3d;
        v3d.reserve(mesh2d.vertices.size());
        for (auto& p : mesh2d.vertices)
            v3d.push_back(uAxis * p.x + vAxis * p.y + extrudeDir);
        for (auto& t : mesh2d.triangles)
            addTri(v3d[t.a], v3d[t.b], v3d[t.c], capNorm);
    }

    // Side faces: for each segment of outer poly, build quad strips
    int nPts = (int)outerPoly.size();
    for (int step = 0; step < extrudeSteps; ++step) {
        double t0 = double(step)     / double(extrudeSteps);
        double t1 = double(step + 1) / double(extrudeSteps);
        Vec3 off0 = extrudeDir * t0;

        for (int i = 0; i < nPts; ++i) {
            int j = (i + 1) % nPts;
            Vec3 b0 = uAxis * outerPoly[i].x + vAxis * outerPoly[i].y + off0;
            Vec3 b1 = uAxis * outerPoly[j].x + vAxis * outerPoly[j].y + off0;
            Vec3 t0p = b0 + extrudeDir * (t1 - t0);
            Vec3 t1p = b1 + extrudeDir * (t1 - t0);
            Vec3 n   = (b1 - b0).cross(t0p - b0);
            double nl = n.norm();
            if (nl > 1e-14) n = n * (1.0 / nl);
            addTri(b0, b1, t1p, n);
            addTri(b0, t1p, t0p, n);
        }
    }

    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// revolveSketch
// ─────────────────────────────────────────────────────────────────────────────

inline ExtrusionMesh3D revolveSketch(const SketchRegion& region,
                                      Vec3   axisOrigin,
                                      Vec3   axisDir,
                                      double angleStart,
                                      double angleEnd,
                                      int    revolutionSteps = 32,
                                      int    samplesPerCurve = 16)
{
    auto outerPoly = region.discretizeOuter(samplesPerCurve);
    int nPts = (int)outerPoly.size();

    Vec3 axN = axisDir.normalized();

    // Build frame: uAxis = in-plane radial start direction, vAxis = in-plane second
    Vec3 uAxis, vAxis2d;
    {
        Vec3 ref = (std::abs(axN.dot(Vec3::unitX())) < 0.9)
                   ? Vec3::unitX() : Vec3::unitY();
        uAxis  = (ref - axN * ref.dot(axN)).normalized();
        vAxis2d = axN.cross(uAxis);
    }

    // Convert 2D sketch point to 3D at a given angle
    // 2D: (x, y) → radial distance = x (from axis), height = y along axis
    // 3D at angle θ: axisOrigin + y*axN + x*(cos(θ)*uAxis + sin(θ)*vAxis2d)
    auto lift = [&](const Vec2& p, double theta) -> Vec3 {
        double ct = std::cos(theta), st = std::sin(theta);
        return axisOrigin + axN * p.y + (uAxis * ct + vAxis2d * st) * p.x;
    };

    ExtrusionMesh3D result;

    auto addTri = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 n) {
        int base = (int)result.vertices.size();
        result.vertices.push_back(a);
        result.vertices.push_back(b);
        result.vertices.push_back(c);
        result.normals.push_back(n);
        result.normals.push_back(n);
        result.normals.push_back(n);
        result.triangles.push_back({base, base+1, base+2});
    };

    double totalAngle = angleEnd - angleStart;
    bool isFullRevolution = (std::abs(totalAngle - 2.0 * 3.14159265358979323846) < 1e-6);

    // Add caps if not a full revolution
    if (!isFullRevolution) {
        // Start cap at angleStart
        std::vector<std::vector<Vec2>> noHoles;
        auto mesh2d = delaunayTriangulate(outerPoly, noHoles);
        Vec3 capNorm = uAxis * std::cos(angleStart) + vAxis2d * std::sin(angleStart);
        capNorm = axN.cross(capNorm).normalized(); // tangent direction = cap normal
        capNorm = capNorm * -1.0; // inward
        std::vector<Vec3> v3d;
        v3d.reserve(mesh2d.vertices.size());
        for (auto& p : mesh2d.vertices)
            v3d.push_back(lift(p, angleStart));
        for (auto& t : mesh2d.triangles)
            addTri(v3d[t.a], v3d[t.b], v3d[t.c], capNorm);

        // End cap at angleEnd
        auto mesh2d2 = delaunayTriangulate(outerPoly, noHoles);
        Vec3 capNorm2 = uAxis * std::cos(angleEnd) + vAxis2d * std::sin(angleEnd);
        capNorm2 = axN.cross(capNorm2).normalized();
        v3d.clear();
        v3d.reserve(mesh2d2.vertices.size());
        for (auto& p : mesh2d2.vertices)
            v3d.push_back(lift(p, angleEnd));
        for (auto& t : mesh2d2.triangles)
            addTri(v3d[t.a], v3d[t.c], v3d[t.b], capNorm2);
    }

    // Side surface: revolve each edge of outer poly
    for (int step = 0; step < revolutionSteps; ++step) {
        double theta0 = angleStart + totalAngle * double(step)     / double(revolutionSteps);
        double theta1 = angleStart + totalAngle * double(step + 1) / double(revolutionSteps);

        for (int i = 0; i < nPts; ++i) {
            int j = (i + 1) % nPts;
            Vec3 p00 = lift(outerPoly[i], theta0);
            Vec3 p10 = lift(outerPoly[j], theta0);
            Vec3 p01 = lift(outerPoly[i], theta1);
            Vec3 p11 = lift(outerPoly[j], theta1);

            Vec3 n = (p10 - p00).cross(p01 - p00);
            double nl = n.norm();
            if (nl > 1e-14) n = n * (1.0 / nl);
            addTri(p00, p10, p11, n);
            addTri(p00, p11, p01, n);
        }
    }

    // Estimate volume using cross-section area * average radius * angle
    {
        double area = detail_sketch::shoelaceArea(outerPoly);
        // Compute centroid x (radial distance from axis)
        double cx = 0.0;
        for (auto& p : outerPoly) cx += p.x;
        cx /= double(nPts);
        result.volume = area * std::abs(totalAngle) * cx;
    }

    return result;
}

} // namespace gk
