// Chapter 5.2 — Torus, Wedge, Prism, Pyramid builders

#include "gk/builders/MoreBuilders.h"
#include "gk/brep/BRepQuery.h"
#include "gk/surface/SurfaceUtils.h"
#include "ObjWriter.h"
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
