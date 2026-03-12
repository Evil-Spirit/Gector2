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

// ─────────────────────────────────────────────────────────────────────────────
// loftSketch  (Chapter 5.4)
// ─────────────────────────────────────────────────────────────────────────────

/// Create a loft mesh between two sketch regions placed in 3D.
/// Both contours are discretized with the same number of points and
/// corresponding vertices are connected by quad strips.  The volume is
/// estimated with the prismatoid formula: h/3 * (A1 + A2 + sqrt(A1*A2)).
inline ExtrusionMesh3D loftSketch(
        const SketchRegion& bottom,
        Vec3 bottomOrigin, Vec3 bottomU, Vec3 bottomV,
        const SketchRegion& top,
        Vec3 topOrigin,    Vec3 topU,    Vec3 topV,
        int samplesPerCurve = 32)
{
    auto bottomPoly = bottom.discretizeOuter(samplesPerCurve);
    auto topPoly    = top.discretizeOuter(samplesPerCurve);

    // Ensure both polygons have the same number of points by resampling
    // the smaller one.  We rebuild the polygon with exactly nPts points
    // spaced uniformly along its perimeter.
    auto resample = [](const std::vector<Vec2>& poly, int n) -> std::vector<Vec2> {
        if ((int)poly.size() == n) return poly;
        std::vector<double> cumLen(poly.size() + 1, 0.0);
        for (int i = 0; i < (int)poly.size(); ++i) {
            int j = (i + 1) % (int)poly.size();
            Vec2 d{poly[j].x - poly[i].x, poly[j].y - poly[i].y};
            cumLen[i + 1] = cumLen[i] + std::sqrt(d.x * d.x + d.y * d.y);
        }
        double total = cumLen.back();
        std::vector<Vec2> result(n);
        for (int k = 0; k < n; ++k) {
            double t = total * double(k) / double(n);
            // find segment
            int seg = 0;
            for (int s = 0; s < (int)poly.size(); ++s) {
                if (cumLen[s + 1] >= t) { seg = s; break; }
            }
            int next = (seg + 1) % (int)poly.size();
            double segLen = cumLen[seg + 1] - cumLen[seg];
            double alpha = (segLen > 1e-14) ? (t - cumLen[seg]) / segLen : 0.0;
            result[k] = Vec2{poly[seg].x + alpha * (poly[next].x - poly[seg].x),
                             poly[seg].y + alpha * (poly[next].y - poly[seg].y)};
        }
        return result;
    };

    int nPts = (int)std::max(bottomPoly.size(), topPoly.size());
    bottomPoly = resample(bottomPoly, nPts);
    topPoly    = resample(topPoly,    nPts);

    auto lift2d = [](const Vec2& p, Vec3 origin, Vec3 u, Vec3 v) -> Vec3 {
        return origin + u * p.x + v * p.y;
    };

    std::vector<Vec3> bottomVerts(nPts), topVerts(nPts);
    for (int i = 0; i < nPts; ++i) {
        bottomVerts[i] = lift2d(bottomPoly[i], bottomOrigin, bottomU, bottomV);
        topVerts[i]    = lift2d(topPoly[i],    topOrigin,    topU,    topV);
    }

    ExtrusionMesh3D result;
    auto addTri = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 n) {
        int base = (int)result.vertices.size();
        result.vertices.push_back(a);
        result.vertices.push_back(b);
        result.vertices.push_back(c);
        result.normals.push_back(n);
        result.normals.push_back(n);
        result.normals.push_back(n);
        result.triangles.push_back({base, base + 1, base + 2});
    };
    auto flatNorm = [](Vec3 a, Vec3 b, Vec3 c) -> Vec3 {
        Vec3 n2 = (b - a).cross(c - a);
        double len = n2.norm();
        return (len > 1e-14) ? n2 * (1.0 / len) : Vec3::zero();
    };

    // Bottom cap (reversed winding so normal points away from the volume)
    {
        std::vector<std::vector<Vec2>> noHoles;
        auto mesh2d = delaunayTriangulate(bottomPoly, noHoles);
        Vec3 capN = bottomU.cross(bottomV).normalized() * -1.0;
        std::vector<Vec3> v3d;
        v3d.reserve(mesh2d.vertices.size());
        for (auto& p : mesh2d.vertices)
            v3d.push_back(lift2d(p, bottomOrigin, bottomU, bottomV));
        for (auto& t : mesh2d.triangles)
            addTri(v3d[t.a], v3d[t.c], v3d[t.b], capN);
    }

    // Top cap
    {
        std::vector<std::vector<Vec2>> noHoles;
        auto mesh2d = delaunayTriangulate(topPoly, noHoles);
        Vec3 capN = topU.cross(topV).normalized();
        std::vector<Vec3> v3d;
        v3d.reserve(mesh2d.vertices.size());
        for (auto& p : mesh2d.vertices)
            v3d.push_back(lift2d(p, topOrigin, topU, topV));
        for (auto& t : mesh2d.triangles)
            addTri(v3d[t.a], v3d[t.b], v3d[t.c], capN);
    }

    // Side quads: connect corresponding edges
    for (int i = 0; i < nPts; ++i) {
        int j = (i + 1) % nPts;
        Vec3 b0 = bottomVerts[i], b1 = bottomVerts[j];
        Vec3 t0 = topVerts[i],   t1 = topVerts[j];
        Vec3 n  = flatNorm(b0, b1, t1);
        addTri(b0, b1, t1, n);
        addTri(b0, t1, t0, n);
    }

    // Volume: prismatoid formula h/3 * (A1 + A2 + sqrt(A1*A2))
    double aBottom = detail_sketch::shoelaceArea(bottomPoly);
    double aTop    = detail_sketch::shoelaceArea(topPoly);
    Vec3   heightDir = topOrigin - bottomOrigin;
    double h = std::abs(heightDir.dot(bottomU.cross(bottomV).normalized()));
    result.volume = h / 3.0 * (aBottom + aTop + std::sqrt(aBottom * aTop));

    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// sweepPipe  (Chapter 5.4)
// ─────────────────────────────────────────────────────────────────────────────

/// Sweep a circular cross-section of radius `circleRadius` along a polyline
/// defined by `spinePoints`.  A parallel-transport frame prevents twist.
/// The computed volume equals π * r² * total_spine_length.
inline ExtrusionMesh3D sweepPipe(double circleRadius,
                                  const std::vector<Vec3>& spinePoints,
                                  int samplesPerCircle = 16)
{
    static constexpr double kPi = 3.14159265358979323846;
    ExtrusionMesh3D result;
    if (spinePoints.size() < 2) return result;

    int n = (int)spinePoints.size();

    // Compute tangents at each spine point
    std::vector<Vec3> tangents(n);
    for (int i = 0; i < n - 1; ++i) {
        Vec3 d = spinePoints[i + 1] - spinePoints[i];
        double len = d.norm();
        tangents[i] = (len > 1e-14) ? d * (1.0 / len) : Vec3::unitZ();
    }
    tangents[n - 1] = tangents[n - 2];

    // Initial frame perpendicular to first tangent
    Vec3 ref = (std::abs(tangents[0].dot(Vec3::unitX())) < 0.9)
               ? Vec3::unitX() : Vec3::unitY();
    Vec3 u0 = (ref - tangents[0] * ref.dot(tangents[0])).normalized();
    Vec3 v0 = tangents[0].cross(u0);

    // Parallel-transport frames along the spine
    std::vector<Vec3> uFrame(n), vFrame(n);
    uFrame[0] = u0;
    vFrame[0] = v0;
    for (int i = 1; i < n; ++i) {
        Vec3 u = uFrame[i - 1] - tangents[i] * uFrame[i - 1].dot(tangents[i]);
        double uLen = u.norm();
        uFrame[i] = (uLen > 1e-14) ? u * (1.0 / uLen) : uFrame[i - 1];
        vFrame[i] = tangents[i].cross(uFrame[i]);
    }

    // Generate a circle at spine index idx
    auto makeCircle = [&](int idx) -> std::vector<Vec3> {
        std::vector<Vec3> pts(samplesPerCircle);
        for (int k = 0; k < samplesPerCircle; ++k) {
            double angle = 2.0 * kPi * double(k) / double(samplesPerCircle);
            pts[k] = spinePoints[idx]
                   + uFrame[idx] * (circleRadius * std::cos(angle))
                   + vFrame[idx] * (circleRadius * std::sin(angle));
        }
        return pts;
    };

    auto addTri = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 norm) {
        int base = (int)result.vertices.size();
        result.vertices.push_back(a);
        result.vertices.push_back(b);
        result.vertices.push_back(c);
        result.normals.push_back(norm);
        result.normals.push_back(norm);
        result.normals.push_back(norm);
        result.triangles.push_back({base, base + 1, base + 2});
    };
    auto flatNorm = [](Vec3 a, Vec3 b, Vec3 c) -> Vec3 {
        Vec3 nm = (b - a).cross(c - a);
        double len = nm.norm();
        return (len > 1e-14) ? nm * (1.0 / len) : Vec3::zero();
    };

    // Start cap (normal = -tangent[0])
    {
        Vec3 cN = tangents[0] * -1.0;
        Vec3 center = spinePoints[0];
        auto pts = makeCircle(0);
        for (int k = 0; k < samplesPerCircle; ++k) {
            int j = (k + 1) % samplesPerCircle;
            addTri(center, pts[j], pts[k], cN);
        }
    }

    // End cap (normal = +tangent[n-1])
    {
        Vec3 cN = tangents[n - 1];
        Vec3 center = spinePoints[n - 1];
        auto pts = makeCircle(n - 1);
        for (int k = 0; k < samplesPerCircle; ++k) {
            int j = (k + 1) % samplesPerCircle;
            addTri(center, pts[k], pts[j], cN);
        }
    }

    // Side faces between consecutive cross-sections
    for (int i = 0; i < n - 1; ++i) {
        auto pts0 = makeCircle(i);
        auto pts1 = makeCircle(i + 1);
        for (int k = 0; k < samplesPerCircle; ++k) {
            int j = (k + 1) % samplesPerCircle;
            Vec3 nm = flatNorm(pts0[k], pts0[j], pts1[j]);
            addTri(pts0[k], pts0[j], pts1[j], nm);
            addTri(pts0[k], pts1[j], pts1[k], nm);
        }
    }

    // Volume = π * r² * total spine length
    result.volume = 0.0;
    for (int i = 0; i < n - 1; ++i)
        result.volume += kPi * circleRadius * circleRadius
                       * (spinePoints[i + 1] - spinePoints[i]).norm();

    return result;
}

} // namespace gk
