#include "GkTest.h"
#include "ObjWriter.h"
#include "gk/surface/Surfaces.h"
#include <cmath>

using namespace gk;

GK_TEST(SurfaceObjWriter, SphereObj) {
    Sphere sphere(Vec3::zero(), 1.0);
    auto mesh = tessellate(sphere, 20, 20);
    ObjWriter obj;
    obj.addSurfaceMesh(mesh);
    obj.write(objOutputPath("sphere_debug.obj"));
    SUCCEED();
}

GK_TEST(SurfaceObjWriter, CylinderObj) {
    Cylinder cyl(Vec3::zero(), Vec3::unitZ(), 1.0, 0.0, 2.0);
    auto mesh = tessellate(cyl, 24, 8);
    ObjWriter obj;
    obj.addSurfaceMesh(mesh);
    obj.write(objOutputPath("cylinder_debug.obj"));
    SUCCEED();
}

GK_TEST(SurfaceObjWriter, TorusObj) {
    Torus torus(Vec3::zero(), Vec3::unitZ(), 2.0, 0.5);
    auto mesh = tessellate(torus, 32, 16);
    ObjWriter obj;
    obj.addSurfaceMesh(mesh);
    obj.write(objOutputPath("torus_debug.obj"));
    SUCCEED();
}

GK_TEST(SurfaceObjWriter, BSplineSurfaceObj) {
    BSplineSurface::CtrlGrid ctrl(4, std::vector<Vec3>(4));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double u = double(i) / 3.0;
            double v = double(j) / 3.0;
            ctrl[i][j] = Vec3(u, v, (u-0.5)*(u-0.5) - (v-0.5)*(v-0.5));
        }
    auto kU = BSplineSurface::uniformKnots(4, 3);
    auto kV = BSplineSurface::uniformKnots(4, 3);
    BSplineSurface surf(3, 3, kU, kV, ctrl);
    auto mesh = tessellate(surf, 16, 16);
    ObjWriter obj;
    obj.addSurfaceMesh(mesh);
    obj.write(objOutputPath("bspline_surface_debug.obj"));
    SUCCEED();
}

GK_TEST(SurfaceObjWriter, TessellationVerticesOnSphere) {
    Sphere sphere(Vec3::zero(), 1.0);
    auto mesh = tessellate(sphere, 20, 20);
    for (auto& v : mesh.vertices) {
        double r = v.norm();
        GK_ASSERT_NEAR(r, 1.0, 1e-6);
    }
}
