// Chapter 5.2 — Torus, Wedge, Prism, Pyramid builders

#include "gk/builders/MoreBuilders.h"
#include "gk/brep/BRepQuery.h"
#include "gk/brep/StepExport.h"
#include "gk/surface/SurfaceUtils.h"
#include "StepWriter.h"
#include <gtest/gtest.h>
#include <cmath>

using namespace gk;

TEST(Builders5_2, TorusVolume) {
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), 2.0, 0.5);
    auto mp = BRepQuery::computeMassProperties(*body, 32);
    constexpr double kPi = 3.14159265358979323846;
    double expected = 2.0 * kPi * kPi * 2.0 * 0.5 * 0.5;
    EXPECT_NEAR(mp.volume, expected, expected * 0.05);
}

TEST(Builders5_2, WedgeMeshVertexCount) {
    auto mesh = makeWedgeMesh(Vec3::zero(), 2.0, 1.0, 1.5);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
}

TEST(Builders5_2, PrismMeshVertexCount) {
    auto mesh = makePrismMesh(Vec3::zero(), Vec3::unitZ(), 1.0, 6, 2.0);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
}

TEST(Builders5_2, PyramidMeshVertexCount) {
    auto mesh = makePyramidMesh(Vec3{0,0,2}, Vec3::zero(), Vec3::unitZ(), 1.0, 4);
    EXPECT_FALSE(mesh.vertices.empty());
    EXPECT_FALSE(mesh.triangles.empty());
}

TEST(Builders5_2, TorusStepExport) {
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), 2.0, 0.5);
    EXPECT_TRUE(StepExport::writeBodyToFile(*body, stepOutputPath("torus_builder_debug.stp")));
}

TEST(Builders5_2, WedgeStepExport) {
    auto mesh = makeWedgeMesh(Vec3::zero(), 2.0, 1.0, 1.5);
    StepWriter step;
    for (auto& v : mesh.vertices) step.addVertex(v);
    for (auto& t : mesh.triangles)
        step.addFace(t[0], t[1], t[2]);
    EXPECT_TRUE(step.write(stepOutputPath("wedge_debug.stp")));
}

TEST(Builders5_2, PrismStepExport) {
    auto mesh = makePrismMesh(Vec3::zero(), Vec3::unitZ(), 1.0, 6, 2.0);
    StepWriter step;
    for (auto& v : mesh.vertices) step.addVertex(v);
    for (auto& t : mesh.triangles)
        step.addFace(t[0], t[1], t[2]);
    EXPECT_TRUE(step.write(stepOutputPath("prism_debug.stp")));
}

TEST(Builders5_2, PyramidStepExport) {
    auto mesh = makePyramidMesh(Vec3{0,0,2}, Vec3::zero(), Vec3::unitZ(), 1.0, 4);
    StepWriter step;
    for (auto& v : mesh.vertices) step.addVertex(v);
    for (auto& t : mesh.triangles)
        step.addFace(t[0], t[1], t[2]);
    EXPECT_TRUE(step.write(stepOutputPath("pyramid_debug.stp")));
}
