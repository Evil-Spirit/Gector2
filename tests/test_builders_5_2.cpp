// Chapter 5.2 — Torus, Wedge, Prism, Pyramid builders

#include "gk/builders/MoreBuilders.h"
#include "gk/brep/BRepQuery.h"
#include "gk/brep/BRep.h"
#include "gk/surface/SurfaceUtils.h"
#include "ObjWriter.h"
#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>

using namespace gk;

static std::string stepOutputPath(const std::string& filename)
{
    std::filesystem::create_directories("step_test_output");
    return "step_test_output/" + filename;
}

TEST(Builders5_2, TorusVolume) {
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), 2.0, 0.5);
    auto mp = BRepQuery::computeMassProperties(*body, 32);
    constexpr double kPi = 3.14159265358979323846;
    double expected = 2.0 * kPi * kPi * 2.0 * 0.5 * 0.5;
    EXPECT_NEAR(mp.volume, expected, expected * 0.05);
}

TEST(Builders5_2, TorusStepExport) {
    // Build torus, verify volume, then write STEP output.
    constexpr double kPi = 3.14159265358979323846;
    double majorR = 2.0, minorR = 0.5;
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), majorR, minorR);
    auto mp = BRepQuery::computeMassProperties(*body, 32);
    double expected = 2.0 * kPi * kPi * majorR * minorR * minorR;
    EXPECT_NEAR(mp.volume, expected, expected * 0.05);

    std::string step = StepWriter::write(*body, "Torus");
    EXPECT_FALSE(step.empty());
    EXPECT_NE(step.find("TOROIDAL_SURFACE"), std::string::npos);
    std::ofstream f(stepOutputPath("torus_volume_check.step"));
    ASSERT_TRUE(f.is_open());
    f << step;
    EXPECT_TRUE(f.good());
}

TEST(Builders5_2, WedgeMeshVertexCount) {
    auto mesh = makeWedgeMesh(Vec3::zero(), 2.0, 1.0, 1.5);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
}

// Volume of a wedge (triangular prism): V = dx * dy * dz / 2
TEST(Builders5_2, WedgeMeshVolume) {
    double dx = 2.0, dy = 1.0, dz = 1.5;
    auto mesh = makeWedgeMesh(Vec3::zero(), dx, dy, dz);
    double vol = meshVolume(mesh);
    double expected = dx * dy * dz / 2.0;
    EXPECT_NEAR(vol, expected, expected * 0.01);
}

TEST(Builders5_2, PrismMeshVertexCount) {
    auto mesh = makePrismMesh(Vec3::zero(), Vec3::unitZ(), 1.0, 6, 2.0);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
}

// Volume of a regular n-sided prism: V = n * r² * sin(2π/n) / 2 * h
TEST(Builders5_2, PrismMeshVolume) {
    constexpr double kPi = 3.14159265358979323846;
    int nSides = 6;
    double r = 1.0, h = 2.0;
    auto mesh = makePrismMesh(Vec3::zero(), Vec3::unitZ(), r, nSides, h);
    double vol = meshVolume(mesh);
    double expected = double(nSides) * r * r * std::sin(2.0 * kPi / double(nSides)) / 2.0 * h;
    EXPECT_NEAR(vol, expected, expected * 0.01);
}

TEST(Builders5_2, PyramidMeshVertexCount) {
    auto mesh = makePyramidMesh(Vec3{0,0,2}, Vec3::zero(), Vec3::unitZ(), 1.0, 4);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
}

// Volume of a regular n-sided pyramid: V = n * r² * sin(2π/n) / 6 * h
TEST(Builders5_2, PyramidMeshVolume) {
    constexpr double kPi = 3.14159265358979323846;
    int nSides = 4;
    double r = 1.0, h = 2.0;
    // apex at (0,0,h), base at origin
    auto mesh = makePyramidMesh(Vec3{0,0,h}, Vec3::zero(), Vec3::unitZ(), r, nSides);
    double vol = meshVolume(mesh);
    double expected = double(nSides) * r * r * std::sin(2.0 * kPi / double(nSides)) / 6.0 * h;
    EXPECT_NEAR(vol, expected, expected * 0.01);
}

TEST(Builders5_2, TorusObjExport) {
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), 2.0, 0.5);
    ObjWriter obj;
    for (auto& lump : body->lumps()) {
        if (!lump || !lump->outerShell()) continue;
        for (auto& face : lump->outerShell()->faces()) {
            if (!face->hasSurface()) continue;
            auto mesh = tessellate(*face->surface(), 32, 16);
            obj.addSurfaceMesh(mesh);
        }
    }
    EXPECT_TRUE(obj.write(objOutputPath("torus_builder_debug.obj")));
}

TEST(Builders5_2, WedgeObjExport) {
    auto mesh = makeWedgeMesh(Vec3::zero(), 2.0, 1.0, 1.5);
    ObjWriter obj;
    for (auto& v : mesh.vertices) obj.addVertex(v);
    for (auto& n : mesh.normals)  obj.addNormal(n);
    int vOff = 1, nOff = 1;
    for (auto& t : mesh.triangles)
        obj.addFaceWithNormals(t[0]+vOff, t[0]+nOff, t[1]+vOff, t[1]+nOff,
                               t[2]+vOff, t[2]+nOff);
    EXPECT_TRUE(obj.write(objOutputPath("wedge_debug.obj")));
}

TEST(Builders5_2, PrismObjExport) {
    auto mesh = makePrismMesh(Vec3::zero(), Vec3::unitZ(), 1.0, 6, 2.0);
    ObjWriter obj;
    for (auto& v : mesh.vertices) obj.addVertex(v);
    for (auto& n : mesh.normals)  obj.addNormal(n);
    int vOff=1, nOff=1;
    for (auto& t : mesh.triangles)
        obj.addFaceWithNormals(t[0]+vOff, t[0]+nOff, t[1]+vOff, t[1]+nOff,
                               t[2]+vOff, t[2]+nOff);
    EXPECT_TRUE(obj.write(objOutputPath("prism_debug.obj")));
}

TEST(Builders5_2, PyramidObjExport) {
    auto mesh = makePyramidMesh(Vec3{0,0,2}, Vec3::zero(), Vec3::unitZ(), 1.0, 4);
    ObjWriter obj;
    for (auto& v : mesh.vertices) obj.addVertex(v);
    for (auto& n : mesh.normals)  obj.addNormal(n);
    int vOff=1, nOff=1;
    for (auto& t : mesh.triangles)
        obj.addFaceWithNormals(t[0]+vOff, t[0]+nOff, t[1]+vOff, t[1]+nOff,
                               t[2]+vOff, t[2]+nOff);
    EXPECT_TRUE(obj.write(objOutputPath("pyramid_debug.obj")));
}
