#pragma once
// Chapter 5 — Sketch Region: a 2D boundary defined by ICurve2 loops.

#include "gk/curve/ICurve.h"
#include "gk/curve/Line.h"
#include "gk/curve/Circle.h"
#include "gk/math/Vec2.h"
#include <cmath>
#include <memory>
#include <vector>

namespace gk {

struct SketchRegion {
    using Loop = std::vector<std::shared_ptr<ICurve2>>;

    Loop              outer;
    std::vector<Loop> holes;

    /// Discretize a loop into 2D sample points.
    static std::vector<Vec2> discretizeLoop(const Loop& loop, int samplesPerCurve = 32)
    {
        std::vector<Vec2> pts;
        pts.reserve(loop.size() * static_cast<std::size_t>(samplesPerCurve));
        for (auto& curve : loop) {
            if (!curve) continue;
            auto dom = curve->domain();
            for (int i = 0; i < samplesPerCurve; ++i) {
                double t = dom.lo + dom.width() * (double(i) / double(samplesPerCurve));
                pts.push_back(curve->evaluate(t).p);
            }
        }
        return pts;
    }

    std::vector<Vec2> discretizeOuter(int samplesPerCurve = 32) const
    {
        return discretizeLoop(outer, samplesPerCurve);
    }

    std::vector<Vec2> discretizeHole(int idx, int samplesPerCurve = 32) const
    {
        return discretizeLoop(holes[static_cast<std::size_t>(idx)], samplesPerCurve);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Factory functions
// ─────────────────────────────────────────────────────────────────────────────

/// Triangle region from three vertices.
inline SketchRegion makeTriangleRegion(Vec2 a, Vec2 b, Vec2 c)
{
    SketchRegion r;
    r.outer.push_back(std::make_shared<Line2>(Line2::fromSegment(a, b)));
    r.outer.push_back(std::make_shared<Line2>(Line2::fromSegment(b, c)));
    r.outer.push_back(std::make_shared<Line2>(Line2::fromSegment(c, a)));
    return r;
}

/// Axis-aligned rectangle region.
inline SketchRegion makeRectangleRegion(Vec2 origin, double width, double height)
{
    Vec2 p0 = origin;
    Vec2 p1{origin.x + width, origin.y};
    Vec2 p2{origin.x + width, origin.y + height};
    Vec2 p3{origin.x,         origin.y + height};
    SketchRegion r;
    r.outer.push_back(std::make_shared<Line2>(Line2::fromSegment(p0, p1)));
    r.outer.push_back(std::make_shared<Line2>(Line2::fromSegment(p1, p2)));
    r.outer.push_back(std::make_shared<Line2>(Line2::fromSegment(p2, p3)));
    r.outer.push_back(std::make_shared<Line2>(Line2::fromSegment(p3, p0)));
    return r;
}

/// Regular polygon region centered at center with circumradius R and n sides.
inline SketchRegion makeRegularPolygonRegion(Vec2 center, double R, int n)
{
    static constexpr double kPi = 3.14159265358979323846;
    SketchRegion r;
    for (int i = 0; i < n; ++i) {
        double a0 = 2.0 * kPi * double(i)     / double(n);
        double a1 = 2.0 * kPi * double(i + 1) / double(n);
        Vec2 p0{center.x + R * std::cos(a0), center.y + R * std::sin(a0)};
        Vec2 p1{center.x + R * std::cos(a1), center.y + R * std::sin(a1)};
        r.outer.push_back(std::make_shared<Line2>(Line2::fromSegment(p0, p1)));
    }
    return r;
}

/// Rounded rectangle: 4 straight sides + 4 quarter-circle corner arcs.
inline SketchRegion makeRoundedRectangleRegion(Vec2 origin, double width,
                                                double height, double cornerR)
{
    static constexpr double kPi = 3.14159265358979323846;
    double x0 = origin.x, y0 = origin.y;
    double x1 = x0 + width, y1 = y0 + height;
    double r  = cornerR;

    // 4 arc centers
    Vec2 cBL{x0 + r, y0 + r};
    Vec2 cBR{x1 - r, y0 + r};
    Vec2 cTR{x1 - r, y1 - r};
    Vec2 cTL{x0 + r, y1 - r};

    SketchRegion reg;
    auto& out = reg.outer;

    // Bottom edge (left to right)
    out.push_back(std::make_shared<Line2>(
        Line2::fromSegment(Vec2{x0 + r, y0}, Vec2{x1 - r, y0})));
    // Bottom-right arc (from bottom to right side), center = cBR, angle -π/2 to 0
    out.push_back(std::make_shared<Circle2>(
        Circle2::arc(cBR, r, -kPi / 2.0, 0.0)));
    // Right edge (bottom to top)
    out.push_back(std::make_shared<Line2>(
        Line2::fromSegment(Vec2{x1, y0 + r}, Vec2{x1, y1 - r})));
    // Top-right arc, center = cTR, angle 0 to π/2
    out.push_back(std::make_shared<Circle2>(
        Circle2::arc(cTR, r, 0.0, kPi / 2.0)));
    // Top edge (right to left)
    out.push_back(std::make_shared<Line2>(
        Line2::fromSegment(Vec2{x1 - r, y1}, Vec2{x0 + r, y1})));
    // Top-left arc, center = cTL, angle π/2 to π
    out.push_back(std::make_shared<Circle2>(
        Circle2::arc(cTL, r, kPi / 2.0, kPi)));
    // Left edge (top to bottom)
    out.push_back(std::make_shared<Line2>(
        Line2::fromSegment(Vec2{x0, y1 - r}, Vec2{x0, y0 + r})));
    // Bottom-left arc, center = cBL, angle π to 3π/2
    out.push_back(std::make_shared<Circle2>(
        Circle2::arc(cBL, r, kPi, 3.0 * kPi / 2.0)));

    return reg;
}

/// Add a circular hole to an existing sketch region.
inline SketchRegion makeCircleHoleRegion(SketchRegion outer, Vec2 holeCenter,
                                          double holeRadius)
{
    static constexpr double kPi = 3.14159265358979323846;
    SketchRegion::Loop hole;
    hole.push_back(std::make_shared<Circle2>(
        Circle2::arc(holeCenter, holeRadius, 0.0, 2.0 * kPi)));
    outer.holes.push_back(std::move(hole));
    return outer;
}

} // namespace gk
