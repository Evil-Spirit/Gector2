// Chapter 5 — Extrusion and Revolution mesh tests

#include "gk/sketch/SketchRegion.h"
#include "gk/sketch/SketchMesh.h"
#include "StepWriter.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace gk;

static void exportMesh(const ExtrusionMesh3D& mesh, const std::string& path)
{
    StepWriter step;
    for (auto& v : mesh.vertices) step.addVertex(v);
    for (auto& t : mesh.triangles)
        step.addFace(t[0], t[1], t[2]);
    step.write(path);
}

static void exportSketchMesh(const SketchMesh3D& mesh, const std::string& path)
{
    StepWriter step;
    for (auto& v : mesh.vertices) step.addVertex(v);
    for (auto& t : mesh.triangles)
        step.addFace(t[0], t[1], t[2]);
    step.write(path);
}

TEST(Extrusion, RectangleExtrude) {
    auto region = makeRectangleRegion(Vec2{0,0}, 2.0, 1.0);
    auto mesh = extrudeSketch(region, Vec3{0,0,3});
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    EXPECT_GT(mesh.volume, 0.0);
    exportMesh(mesh, stepOutputPath("extrude_rectangle.stp"));
}

TEST(Extrusion, TriangleExtrude) {
    auto region = makeTriangleRegion(Vec2{0,0}, Vec2{2,0}, Vec2{1,1.7});
    auto mesh = extrudeSketch(region, Vec3{0,0,2});
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    exportMesh(mesh, stepOutputPath("extrude_triangle.stp"));
}

TEST(Extrusion, HexagonExtrude) {
    auto region = makeRegularPolygonRegion(Vec2{0,0}, 1.0, 6);
    auto mesh = extrudeSketch(region, Vec3{0,0,2});
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    exportMesh(mesh, stepOutputPath("extrude_hexagon.stp"));
}

TEST(Extrusion, RectangleRevolve) {
    // Revolve a rectangle around Y axis (full revolution = cylinder-ish)
    // Rectangle at x=1..2, y=0..1 → torus-like shape
    auto region = makeRectangleRegion(Vec2{1.5, 0.0}, 0.3, 1.0);
    constexpr double kPi = 3.14159265358979323846;
    auto mesh = revolveSketch(region, Vec3::zero(), Vec3::unitY(),
                               0.0, 2.0*kPi, 32, 8);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    exportMesh(mesh, stepOutputPath("revolve_rectangle.stp"));
}

TEST(Extrusion, TriangleRevolve) {
    auto region = makeTriangleRegion(Vec2{0.5,0}, Vec2{1.5,0}, Vec2{1.0,1.0});
    constexpr double kPi = 3.14159265358979323846;
    auto mesh = revolveSketch(region, Vec3::zero(), Vec3::unitZ(),
                               0.0, 2.0*kPi, 32, 8);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    exportMesh(mesh, stepOutputPath("revolve_triangle.stp"));
}

TEST(Extrusion, TriangulateSketch) {
    auto region = makeRectangleRegion(Vec2{0,0}, 2.0, 1.0);
    auto mesh = triangulateSketch(region, Vec3::zero(), Vec3::unitX(), Vec3::unitY(), 8);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
    exportSketchMesh(mesh, stepOutputPath("sketch_mesh_rectangle.stp"));
}

TEST(Extrusion, ExtrudeVolumeApprox) {
    // A 1x1 rectangle extruded by 2 should have volume ~2
    auto region = makeRectangleRegion(Vec2{0,0}, 1.0, 1.0);
    auto mesh = extrudeSketch(region, Vec3{0,0,2}, 8, 1);
    EXPECT_NEAR(mesh.volume, 2.0, 0.01);
}
