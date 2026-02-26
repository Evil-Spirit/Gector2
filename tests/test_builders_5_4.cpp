// Chapter 5.4 — Loft & Pipe Sweep builders

#include "gk/sketch/SketchMesh.h"
#include "gk/sketch/SketchRegion.h"
#include "ObjWriter.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace gk;

static constexpr double kPi = 3.14159265358979323846;

// ── Helper: export ExtrusionMesh3D to OBJ ─────────────────────────────────────

static void exportMesh(const ExtrusionMesh3D& mesh, const std::string& path)
{
    ObjWriter obj;
    for (auto& v : mesh.vertices) obj.addVertex(v);
    for (auto& n : mesh.normals)  obj.addNormal(n);
    int vOff = 1, nOff = 1;
    for (auto& t : mesh.triangles)
        obj.addFaceWithNormals(t[0]+vOff, t[0]+nOff,
                               t[1]+vOff, t[1]+nOff,
                               t[2]+vOff, t[2]+nOff);
    obj.write(path);
}

// ── Loft tests ────────────────────────────────────────────────────────────────

// Lofting two identical rectangles at different heights equals extrusion.
TEST(Builders5_4, LoftIdenticalRectangles_EqualsExtrusion) {
    double w = 2.0, h = 1.0, height = 3.0;
    auto region = makeRectangleRegion(Vec2{0, 0}, w, h);
    auto mesh = loftSketch(
        region, Vec3::zero(), Vec3::unitX(), Vec3::unitY(),
        region, Vec3{0, 0, height}, Vec3::unitX(), Vec3::unitY());
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    EXPECT_NEAR(mesh.volume, w * h * height, w * h * height * 0.02);
}

// Loft between two different-sized rectangles: use prismatoid formula.
TEST(Builders5_4, LoftRectangleToSmallerRectangle) {
    double w1 = 2.0, h1 = 2.0;  // bottom: 2×2
    double w2 = 1.0, h2 = 1.0;  // top:    1×1
    double ht = 3.0;
    auto bottom = makeRectangleRegion(Vec2{0, 0}, w1, h1);
    auto top    = makeRectangleRegion(Vec2{0, 0}, w2, h2);
    auto mesh   = loftSketch(
        bottom, Vec3::zero(),   Vec3::unitX(), Vec3::unitY(),
        top,    Vec3{0,0,ht},   Vec3::unitX(), Vec3::unitY());
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    // Prismatoid: V = h/3 * (A1 + A2 + sqrt(A1*A2))
    double a1 = w1 * h1, a2 = w2 * h2;
    double expected = ht / 3.0 * (a1 + a2 + std::sqrt(a1 * a2));
    EXPECT_NEAR(mesh.volume, expected, expected * 0.05);
}

// Loft between a rectangle and a regular hexagon.
TEST(Builders5_4, LoftRectangleToHexagon) {
    auto bottom = makeRectangleRegion(Vec2{-1, -1}, 2.0, 2.0);
    auto top    = makeRegularPolygonRegion(Vec2{0, 0}, 1.0, 6);
    auto mesh   = loftSketch(
        bottom, Vec3::zero(),  Vec3::unitX(), Vec3::unitY(),
        top,    Vec3{0,0,2.0}, Vec3::unitX(), Vec3::unitY());
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    EXPECT_GT(mesh.volume, 0.0);
}

TEST(Builders5_4, LoftObjExport) {
    auto bottom = makeRectangleRegion(Vec2{0, 0}, 2.0, 2.0);
    auto top    = makeRegularPolygonRegion(Vec2{0.5, 0.5}, 0.5, 6);
    auto mesh   = loftSketch(
        bottom, Vec3::zero(),    Vec3::unitX(), Vec3::unitY(),
        top,    Vec3{0, 0, 3.0}, Vec3::unitX(), Vec3::unitY());
    EXPECT_FALSE(mesh.vertices.empty());
    exportMesh(mesh, objOutputPath("loft_rect_to_hex.obj"));
}

// ── Pipe Sweep tests ──────────────────────────────────────────────────────────

// Straight pipe along Z equals a cylinder: V = π * r² * length.
TEST(Builders5_4, PipeStraightSpine_EqualsCylinder) {
    double r = 1.0, len = 3.0;
    std::vector<Vec3> spine = {Vec3{0,0,0}, Vec3{0,0,1}, Vec3{0,0,2}, Vec3{0,0,len}};
    auto mesh = sweepPipe(r, spine, 32);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    double expected = kPi * r * r * len;
    EXPECT_NEAR(mesh.volume, expected, expected * 0.02);
}

// Pipe with two segments: total volume = π * r² * total_length.
TEST(Builders5_4, PipeLSpine) {
    double r = 0.5;
    std::vector<Vec3> spine = {Vec3{0,0,0}, Vec3{2,0,0}, Vec3{2,3,0}};
    auto mesh = sweepPipe(r, spine, 16);
    EXPECT_FALSE(mesh.vertices.empty());
    double totalLen = 2.0 + 3.0;
    double expected = kPi * r * r * totalLen;
    EXPECT_NEAR(mesh.volume, expected, expected * 0.02);
}

// Pipe along a curved (staircase) spine: volume = π * r² * total_length.
TEST(Builders5_4, PipeCurvedSpine) {
    double r = 0.3;
    std::vector<Vec3> spine;
    int nSteps = 16;
    for (int i = 0; i <= nSteps; ++i) {
        double t = double(i) / double(nSteps);
        spine.push_back(Vec3{std::cos(t * kPi), std::sin(t * kPi), t});
    }
    auto mesh = sweepPipe(r, spine, 16);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_GT(mesh.volume, 0.0);
}

TEST(Builders5_4, PipeObjExport) {
    double r = 0.25;
    std::vector<Vec3> spine;
    int nSteps = 32;
    for (int i = 0; i <= nSteps; ++i) {
        double t = double(i) / double(nSteps);
        spine.push_back(Vec3{2.0 * std::cos(2.0 * kPi * t),
                              2.0 * std::sin(2.0 * kPi * t),
                              t});
    }
    auto mesh = sweepPipe(r, spine, 16);
    EXPECT_FALSE(mesh.vertices.empty());
    exportMesh(mesh, objOutputPath("pipe_helix.obj"));
}
