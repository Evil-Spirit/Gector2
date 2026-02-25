// Chapter 5 — Sketch region triangulation tests

#include "gk/sketch/SketchRegion.h"
#include "gk/sketch/Delaunay2D.h"
#include "ObjWriter.h"
#include "SvgWriter.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace gk;

static void exportSketchObj(const std::vector<Vec2>& pts,
                             const std::vector<DelauTri>& tris,
                             const std::string& path)
{
    ObjWriter obj;
    for (auto& p : pts)
        obj.addVertex(Vec3{p.x, p.y, 0.0});
    int vOff = 1;
    for (auto& t : tris)
        obj.addFace(t.a + vOff, t.b + vOff, t.c + vOff);
    obj.write(path);
}

static void exportSketchSvg(const std::vector<Vec2>& outerPts,
                              const std::string& path)
{
    SvgWriter svg(400, 400);
    double minX = outerPts[0].x, maxX = outerPts[0].x;
    double minY = outerPts[0].y, maxY = outerPts[0].y;
    for (auto& p : outerPts) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    double pad = (maxX - minX + maxY - minY) * 0.1 + 0.1;
    svg.setView(minX - pad, minY - pad, maxX + pad, maxY + pad);
    auto closed = outerPts;
    closed.push_back(outerPts[0]);
    svg.addPolyline(closed);
    svg.write(path);
}

TEST(Sketch, TriangleTriangulate) {
    auto region = makeTriangleRegion(Vec2{0,0}, Vec2{2,0}, Vec2{1,1.7});
    auto poly = region.discretizeOuter(4);
    auto mesh = delaunayTriangulate(poly);
    EXPECT_GT((int)mesh.triangles.size(), 0);
    exportSketchObj(mesh.vertices, mesh.triangles, objOutputPath("sketch_triangle.obj"));
    exportSketchSvg(poly, svgOutputPath("sketch_triangle.svg"));
}

TEST(Sketch, RectangleTriangulate) {
    auto region = makeRectangleRegion(Vec2{0,0}, 3.0, 2.0);
    auto poly = region.discretizeOuter(4);
    auto mesh = delaunayTriangulate(poly);
    EXPECT_GT((int)mesh.triangles.size(), 0);
    exportSketchObj(mesh.vertices, mesh.triangles, objOutputPath("sketch_rectangle.obj"));
    exportSketchSvg(poly, svgOutputPath("sketch_rectangle.svg"));
}

TEST(Sketch, HexagonTriangulate) {
    auto region = makeRegularPolygonRegion(Vec2{0,0}, 1.0, 6);
    auto poly = region.discretizeOuter(4);
    auto mesh = delaunayTriangulate(poly);
    EXPECT_GT((int)mesh.triangles.size(), 0);
    exportSketchObj(mesh.vertices, mesh.triangles, objOutputPath("sketch_hexagon.obj"));
}

TEST(Sketch, HexWithHole) {
    auto hexRegion = makeRegularPolygonRegion(Vec2{0,0}, 1.0, 6);
    auto nutRegion = makeCircleHoleRegion(hexRegion, Vec2{0,0}, 0.4);
    auto outerPoly = nutRegion.discretizeOuter(8);
    std::vector<std::vector<Vec2>> holePoly;
    holePoly.push_back(nutRegion.discretizeHole(0, 16));
    auto mesh = delaunayTriangulate(outerPoly, holePoly);
    EXPECT_GT((int)mesh.triangles.size(), 0);
    exportSketchObj(mesh.vertices, mesh.triangles, objOutputPath("sketch_hex_hole.obj"));
}

TEST(Sketch, RoundedRectangle) {
    auto region = makeRoundedRectangleRegion(Vec2{0,0}, 3.0, 2.0, 0.3);
    auto poly = region.discretizeOuter(8);
    EXPECT_GT((int)poly.size(), 0);
    auto mesh = delaunayTriangulate(poly);
    EXPECT_GT((int)mesh.triangles.size(), 0);
    exportSketchObj(mesh.vertices, mesh.triangles, objOutputPath("sketch_rounded_rect.obj"));
    exportSketchSvg(poly, svgOutputPath("sketch_rounded_rect.svg"));
}

TEST(Sketch, DiscretizeLoopNonEmpty) {
    auto region = makeRectangleRegion(Vec2{0,0}, 1.0, 1.0);
    auto pts = region.discretizeOuter(8);
    EXPECT_EQ((int)pts.size(), 4 * 8);
}
