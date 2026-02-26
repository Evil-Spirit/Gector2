// Chapter 9.1 — StepWriter tests.
//
// Verifies that gk::StepWriter produces valid ISO 10303-21 STEP output for:
//   1. Pure topological B-Rep (cube with edge/vertex connectivity)
//   2. Box from PrimitiveBuilders (Plane surfaces)
//   3. Sphere from PrimitiveBuilders (SPHERICAL_SURFACE)
//   4. Cylinder from PrimitiveBuilders (CYLINDRICAL_SURFACE + Disc/PLANE)
//   5. Cone from PrimitiveBuilders (CONICAL_SURFACE + Disc/PLANE)
//   6. Torus from MoreBuilders (TOROIDAL_SURFACE)
//   7. B-Rep with BSpline curve-bound edges
//   8. B-Rep with BSplineSurface face
//   9. B-Rep with NURBSSurface face
//  10. STEP output written to file

#include "gk/brep/BRep.h"
#include "gk/builders/PrimitiveBuilders.h"
#include "gk/builders/MoreBuilders.h"
#include "gk/surface/BSplineSurface.h"
#include "gk/surface/NURBSSurface.h"
#include "gk/curve/BSplineCurve.h"
#include "gk/curve/NURBSCurve.h"
#include "gk/curve/Line.h"
#include "gk/curve/Circle.h"
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

using namespace gk;

// ── Helper: return output directory path, creating it if absent ──────────────

static std::string stepOutputPath(const std::string& filename)
{
    std::filesystem::create_directories("step_test_output");
    return "step_test_output/" + filename;
}

// ── Helpers: string checks ────────────────────────────────────────────────────

static bool contains(const std::string& haystack, const std::string& needle)
{
    return haystack.find(needle) != std::string::npos;
}

// ── Minimal cube builder (topology only, same as test_brep_io.cpp) ────────────

static Handle<Body> makeTopoCube()
{
    Handle<Vertex> v[8];
    v[0] = makeHandle<Vertex>(Vec3{0,0,0});
    v[1] = makeHandle<Vertex>(Vec3{1,0,0});
    v[2] = makeHandle<Vertex>(Vec3{1,1,0});
    v[3] = makeHandle<Vertex>(Vec3{0,1,0});
    v[4] = makeHandle<Vertex>(Vec3{0,0,1});
    v[5] = makeHandle<Vertex>(Vec3{1,0,1});
    v[6] = makeHandle<Vertex>(Vec3{1,1,1});
    v[7] = makeHandle<Vertex>(Vec3{0,1,1});

    Handle<Edge> e[12];
    e[0]  = makeHandle<Edge>(v[0], v[1]);
    e[1]  = makeHandle<Edge>(v[1], v[2]);
    e[2]  = makeHandle<Edge>(v[2], v[3]);
    e[3]  = makeHandle<Edge>(v[3], v[0]);
    e[4]  = makeHandle<Edge>(v[4], v[5]);
    e[5]  = makeHandle<Edge>(v[5], v[6]);
    e[6]  = makeHandle<Edge>(v[6], v[7]);
    e[7]  = makeHandle<Edge>(v[7], v[4]);
    e[8]  = makeHandle<Edge>(v[0], v[4]);
    e[9]  = makeHandle<Edge>(v[1], v[5]);
    e[10] = makeHandle<Edge>(v[2], v[6]);
    e[11] = makeHandle<Edge>(v[3], v[7]);

    auto makeQuad = [](Handle<Edge> e0, CoEdgeOrientation o0,
                       Handle<Edge> e1, CoEdgeOrientation o1,
                       Handle<Edge> e2, CoEdgeOrientation o2,
                       Handle<Edge> e3, CoEdgeOrientation o3) -> Handle<Wire>
    {
        auto w = makeHandle<Wire>();
        w->addCoEdge(makeHandle<CoEdge>(e0, o0));
        w->addCoEdge(makeHandle<CoEdge>(e1, o1));
        w->addCoEdge(makeHandle<CoEdge>(e2, o2));
        w->addCoEdge(makeHandle<CoEdge>(e3, o3));
        return w;
    };
    using O = CoEdgeOrientation;

    auto fBottom = makeHandle<Face>();
    fBottom->setOuterWire(makeQuad(e[0],O::kForward,  e[1],O::kForward,
                                    e[2],O::kForward,  e[3],O::kForward));
    auto fTop = makeHandle<Face>();
    fTop->setOuterWire(makeQuad(e[4],O::kForward,  e[5],O::kForward,
                                 e[6],O::kForward,  e[7],O::kForward));
    auto fFront = makeHandle<Face>();
    fFront->setOuterWire(makeQuad(e[0],O::kForward,  e[9],O::kForward,
                                   e[4],O::kReversed, e[8],O::kReversed));
    auto fBack = makeHandle<Face>();
    fBack->setOuterWire(makeQuad(e[2],O::kForward,  e[11],O::kForward,
                                  e[6],O::kReversed, e[10],O::kReversed));
    auto fLeft = makeHandle<Face>();
    fLeft->setOuterWire(makeQuad(e[3],O::kReversed, e[11],O::kForward,
                                  e[7],O::kForward,  e[8],O::kReversed));
    auto fRight = makeHandle<Face>();
    fRight->setOuterWire(makeQuad(e[1],O::kForward,  e[10],O::kForward,
                                   e[5],O::kReversed, e[9],O::kReversed));

    auto shell = makeHandle<Shell>();
    shell->setClosed(true);
    shell->addFace(fBottom); shell->addFace(fTop);
    shell->addFace(fFront);  shell->addFace(fBack);
    shell->addFace(fLeft);   shell->addFace(fRight);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);

    auto body = makeHandle<Body>();
    body->addLump(lump);
    return body;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

// 1. Basic structural checks ──────────────────────────────────────────────────

TEST(StepWriterTest, OutputNotEmpty)
{
    auto body = makeTopoCube();
    std::string step = StepWriter::write(*body, "TopoCube");
    EXPECT_FALSE(step.empty());
}

TEST(StepWriterTest, HeaderAndFooter)
{
    auto body = makeTopoCube();
    std::string step = StepWriter::write(*body);
    EXPECT_TRUE(contains(step, "ISO-10303-21;"));
    EXPECT_TRUE(contains(step, "HEADER;"));
    EXPECT_TRUE(contains(step, "FILE_SCHEMA"));
    EXPECT_TRUE(contains(step, "DATA;"));
    EXPECT_TRUE(contains(step, "ENDSEC;"));
    EXPECT_TRUE(contains(step, "END-ISO-10303-21;"));
}

TEST(StepWriterTest, ContainsProductEntities)
{
    auto body = makeTopoCube();
    std::string step = StepWriter::write(*body, "TestPart");
    EXPECT_TRUE(contains(step, "APPLICATION_CONTEXT"));
    EXPECT_TRUE(contains(step, "PRODUCT("));
    EXPECT_TRUE(contains(step, "PRODUCT_DEFINITION("));
    EXPECT_TRUE(contains(step, "ADVANCED_BREP_SHAPE_REPRESENTATION"));
    EXPECT_TRUE(contains(step, "SHAPE_DEFINITION_REPRESENTATION"));
}

TEST(StepWriterTest, ContainsBRepEntities)
{
    auto body = makeTopoCube();
    std::string step = StepWriter::write(*body);
    EXPECT_TRUE(contains(step, "MANIFOLD_SOLID_BREP"));
    EXPECT_TRUE(contains(step, "CLOSED_SHELL"));
    EXPECT_TRUE(contains(step, "ADVANCED_FACE"));
    EXPECT_TRUE(contains(step, "FACE_OUTER_BOUND"));
    EXPECT_TRUE(contains(step, "EDGE_LOOP"));
    EXPECT_TRUE(contains(step, "ORIENTED_EDGE"));
    EXPECT_TRUE(contains(step, "EDGE_CURVE"));
    EXPECT_TRUE(contains(step, "VERTEX_POINT"));
    EXPECT_TRUE(contains(step, "CARTESIAN_POINT"));
    EXPECT_TRUE(contains(step, "LINE"));
}

// 2. PrimitiveBuilders solid types ────────────────────────────────────────────

TEST(StepWriterTest, BoxPlaneSurfaces)
{
    auto body = makeBox(Vec3::zero(), Vec3{1, 1, 1});
    std::string step = StepWriter::write(*body, "Box");
    EXPECT_TRUE(contains(step, "ADVANCED_BREP_SHAPE_REPRESENTATION"));
    EXPECT_TRUE(contains(step, "PLANE("));
    EXPECT_TRUE(contains(step, "AXIS2_PLACEMENT_3D"));
}

TEST(StepWriterTest, SphereSphericalSurface)
{
    auto body = makeSphere(Vec3::zero(), 1.0);
    std::string step = StepWriter::write(*body, "Sphere");
    EXPECT_TRUE(contains(step, "SPHERICAL_SURFACE"));
}

TEST(StepWriterTest, CylinderCylindricalSurface)
{
    auto body = makeCylinder(Vec3::zero(), Vec3::unitZ(), 1.0, 2.0);
    std::string step = StepWriter::write(*body, "Cylinder");
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    // Cylinder also has two DiscSurface caps, exported as PLANE
    EXPECT_TRUE(contains(step, "PLANE("));
}

TEST(StepWriterTest, ConeConicalSurface)
{
    constexpr double kPi = 3.14159265358979323846;
    auto body = makeCone(Vec3::zero(), Vec3::unitZ(), kPi / 6.0, 2.0);
    std::string step = StepWriter::write(*body, "Cone");
    EXPECT_TRUE(contains(step, "CONICAL_SURFACE"));
    EXPECT_TRUE(contains(step, "PLANE("));
}

TEST(StepWriterTest, TorusToroidalSurface)
{
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), 2.0, 0.5);
    std::string step = StepWriter::write(*body, "Torus");
    EXPECT_TRUE(contains(step, "TOROIDAL_SURFACE"));
}

// 3. Edges with explicit curve binding ────────────────────────────────────────

TEST(StepWriterTest, EdgeWithLine3Curve)
{
    // Build a single face with two edges carrying Line3 curves
    auto v0 = makeHandle<Vertex>(Vec3{0, 0, 0});
    auto v1 = makeHandle<Vertex>(Vec3{1, 0, 0});
    auto v2 = makeHandle<Vertex>(Vec3{1, 1, 0});
    auto v3 = makeHandle<Vertex>(Vec3{0, 1, 0});

    auto makeLinearEdge = [](Handle<Vertex> s, Handle<Vertex> e2) {
        auto edge = makeHandle<Edge>(s, e2);
        auto line = std::make_shared<Line3>(
            s->point(),
            e2->point() - s->point(),
            0.0, 1.0);
        edge->setCurve(line, 0.0, 1.0);
        return edge;
    };

    auto e0 = makeLinearEdge(v0, v1);
    auto e1 = makeLinearEdge(v1, v2);
    auto e2 = makeLinearEdge(v2, v3);
    auto e3 = makeLinearEdge(v3, v0);

    using O = CoEdgeOrientation;
    auto wire = makeHandle<Wire>();
    wire->addCoEdge(makeHandle<CoEdge>(e0, O::kForward));
    wire->addCoEdge(makeHandle<CoEdge>(e1, O::kForward));
    wire->addCoEdge(makeHandle<CoEdge>(e2, O::kForward));
    wire->addCoEdge(makeHandle<CoEdge>(e3, O::kForward));

    auto face = makeHandle<Face>();
    face->setSurface(std::make_shared<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY()));
    face->setOuterWire(wire);

    auto shell = makeHandle<Shell>();
    shell->setClosed(false);
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);

    auto body = makeHandle<Body>();
    body->addLump(lump);

    std::string step = StepWriter::write(*body, "QuadFace");
    EXPECT_TRUE(contains(step, "LINE("));
    EXPECT_TRUE(contains(step, "PLANE("));
    EXPECT_TRUE(contains(step, "EDGE_CURVE"));
    EXPECT_TRUE(contains(step, "ORIENTED_EDGE"));
}

TEST(StepWriterTest, EdgeWithCircle3Curve)
{
    constexpr double kPi = 3.14159265358979323846;
    auto v0 = makeHandle<Vertex>(Vec3{1, 0, 0});
    auto e0 = makeHandle<Edge>(v0, v0); // closed loop
    auto circle = std::make_shared<Circle3>(
        Vec3::zero(), 1.0, Vec3::unitX(), Vec3::unitY());
    e0->setCurve(circle, 0.0, 2.0 * kPi);

    using O = CoEdgeOrientation;
    auto wire = makeHandle<Wire>();
    wire->addCoEdge(makeHandle<CoEdge>(e0, O::kForward));

    auto face = makeHandle<Face>();
    face->setSurface(std::make_shared<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY()));
    face->setOuterWire(wire);

    auto shell = makeHandle<Shell>();
    shell->setClosed(false);
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);

    auto body = makeHandle<Body>();
    body->addLump(lump);

    std::string step = StepWriter::write(*body, "CircleFace");
    EXPECT_TRUE(contains(step, "CIRCLE("));
    EXPECT_TRUE(contains(step, "AXIS2_PLACEMENT_3D"));
}

// 4. BSpline curve edges ───────────────────────────────────────────────────────

TEST(StepWriterTest, EdgeWithBSplineCurve)
{
    // Cubic BSpline arc approximating a quarter circle
    std::vector<Vec3> ctrl = {
        Vec3{1, 0, 0}, Vec3{1, 0.5, 0}, Vec3{0.5, 1, 0}, Vec3{0, 1, 0}
    };
    auto knots = BSplineCurve3::uniformKnots((int)ctrl.size(), 3);
    auto bspline = std::make_shared<BSplineCurve3>(3, knots, ctrl);

    auto v0 = makeHandle<Vertex>(Vec3{1, 0, 0});
    auto v1 = makeHandle<Vertex>(Vec3{0, 1, 0});
    auto edge = makeHandle<Edge>(v0, v1);
    edge->setCurve(bspline, knots.front(), knots.back());

    using O = CoEdgeOrientation;
    auto wire = makeHandle<Wire>();
    wire->addCoEdge(makeHandle<CoEdge>(edge, O::kForward));

    auto face = makeHandle<Face>();
    face->setSurface(std::make_shared<Plane>(Vec3::zero(), Vec3::unitX(), Vec3::unitY()));
    face->setOuterWire(wire);

    auto shell = makeHandle<Shell>();
    shell->setClosed(false);
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);

    auto body = makeHandle<Body>();
    body->addLump(lump);

    std::string step = StepWriter::write(*body, "BSplineCurveFace");
    EXPECT_TRUE(contains(step, "B_SPLINE_CURVE_WITH_KNOTS"));
}

// 5. BSpline and NURBS surface faces ──────────────────────────────────────────

TEST(StepWriterTest, BSplineSurfaceFace)
{
    // 3×3 bicubic B-spline surface (degree 2 in both directions)
    int nu = 3, nv = 3;
    BSplineSurface::CtrlGrid ctrl(nu, std::vector<Vec3>(nv));
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j)
            ctrl[i][j] = Vec3{double(i), double(j), 0.5 * double(i + j)};

    auto knotsU = BSplineSurface::uniformKnots(nu, 2);
    auto knotsV = BSplineSurface::uniformKnots(nv, 2);
    auto surf = std::make_shared<BSplineSurface>(2, 2, knotsU, knotsV, ctrl);

    auto face = makeHandle<Face>();
    face->setSurface(surf);
    face->setOuterWire(makeHandle<Wire>());

    auto shell = makeHandle<Shell>();
    shell->setClosed(false);
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);

    auto body = makeHandle<Body>();
    body->addLump(lump);

    std::string step = StepWriter::write(*body, "BSplineSurf");
    EXPECT_TRUE(contains(step, "B_SPLINE_SURFACE_WITH_KNOTS"));
}

TEST(StepWriterTest, NURBSSurfaceFace)
{
    // 2×2 NURBS surface (degree 1 in both directions, non-unit weights)
    int nu = 2, nv = 2;
    NURBSSurface::CtrlGrid ctrl = {
        {Vec3{0, 0, 0}, Vec3{0, 1, 0}},
        {Vec3{1, 0, 0}, Vec3{1, 1, 1}}
    };
    NURBSSurface::WtGrid wt = {{1.0, 0.7071}, {0.7071, 1.0}};
    auto knotsU = BSplineSurface::uniformKnots(nu, 1);
    auto knotsV = BSplineSurface::uniformKnots(nv, 1);
    auto surf = std::make_shared<NURBSSurface>(1, 1, knotsU, knotsV, ctrl, wt);

    auto face = makeHandle<Face>();
    face->setSurface(surf);
    face->setOuterWire(makeHandle<Wire>());

    auto shell = makeHandle<Shell>();
    shell->setClosed(false);
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);

    auto body = makeHandle<Body>();
    body->addLump(lump);

    std::string step = StepWriter::write(*body, "NURBSSurf");
    EXPECT_TRUE(contains(step, "RATIONAL_B_SPLINE_SURFACE"));
    EXPECT_TRUE(contains(step, "B_SPLINE_SURFACE_WITH_KNOTS"));
}

// 6. File output ───────────────────────────────────────────────────────────────

// Helper: confirm there are no empty EDGE_LOOP('',()) in the output
static void assertNoEmptyEdgeLoops(const std::string& step)
{
    // An empty EDGE_LOOP looks like: EDGE_LOOP('',())
    EXPECT_EQ(step.find("EDGE_LOOP('',())"), std::string::npos)
        << "Found empty EDGE_LOOP('',()) — face boundary is missing";
}

TEST(StepWriterTest, BoxHasNonEmptyEdgeLoops)
{
    auto body = makeBox(Vec3::zero(), Vec3{1, 1, 1});
    std::string step = StepWriter::write(*body);
    assertNoEmptyEdgeLoops(step);
    // A box has 6 faces each with 4 LINE edges
    EXPECT_TRUE(contains(step, "LINE("));
    // Verify: 6 ADVANCED_FACE entries
    std::size_t faceCnt = 0, pos = 0;
    while ((pos = step.find("ADVANCED_FACE", pos)) != std::string::npos) {
        ++faceCnt;  pos += 13;
    }
    EXPECT_EQ(faceCnt, static_cast<std::size_t>(6));
}

TEST(StepWriterTest, SphereHasNonEmptyEdgeLoops)
{
    auto body = makeSphere(Vec3::zero(), 1.0);
    std::string step = StepWriter::write(*body);
    assertNoEmptyEdgeLoops(step);
    EXPECT_TRUE(contains(step, "CIRCLE("));
}

TEST(StepWriterTest, CylinderHasNonEmptyEdgeLoops)
{
    auto body = makeCylinder(Vec3::zero(), Vec3::unitZ(), 1.0, 2.0);
    std::string step = StepWriter::write(*body);
    assertNoEmptyEdgeLoops(step);
    EXPECT_TRUE(contains(step, "CIRCLE("));
    EXPECT_TRUE(contains(step, "LINE("));
}

TEST(StepWriterTest, ConeHasNonEmptyEdgeLoops)
{
    constexpr double kPi = 3.14159265358979323846;
    auto body = makeCone(Vec3::zero(), Vec3::unitZ(), kPi / 6.0, 2.0);
    std::string step = StepWriter::write(*body);
    assertNoEmptyEdgeLoops(step);
    EXPECT_TRUE(contains(step, "CIRCLE("));
    EXPECT_TRUE(contains(step, "LINE("));
}

TEST(StepWriterTest, TorusHasNonEmptyEdgeLoops)
{
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), 2.0, 0.5);
    std::string step = StepWriter::write(*body);
    assertNoEmptyEdgeLoops(step);
    EXPECT_TRUE(contains(step, "CIRCLE("));
}

TEST(StepWriterTest, WriteToFile_TopoCube)
{
    auto body = makeTopoCube();
    std::string step = StepWriter::write(*body, "TopoCube");
    std::string path = stepOutputPath("topo_cube.step");
    std::ofstream f(path);
    ASSERT_TRUE(f.is_open()) << "Cannot open " << path;
    f << step;
    EXPECT_TRUE(f.good());
}

TEST(StepWriterTest, WriteToFile_Box)
{
    auto body = makeBox(Vec3::zero(), Vec3{2, 1, 1.5});
    std::string step = StepWriter::write(*body, "Box");
    std::string path = stepOutputPath("box.step");
    std::ofstream f(path);
    ASSERT_TRUE(f.is_open());
    f << step;
    EXPECT_TRUE(f.good());
}

TEST(StepWriterTest, WriteToFile_Sphere)
{
    auto body = makeSphere(Vec3::zero(), 1.5);
    std::string step = StepWriter::write(*body, "Sphere");
    std::ofstream f(stepOutputPath("sphere.step"));
    ASSERT_TRUE(f.is_open());
    f << step;
    EXPECT_TRUE(f.good());
}

TEST(StepWriterTest, WriteToFile_Cylinder)
{
    auto body = makeCylinder(Vec3::zero(), Vec3::unitZ(), 1.0, 3.0);
    std::string step = StepWriter::write(*body, "Cylinder");
    std::ofstream f(stepOutputPath("cylinder.step"));
    ASSERT_TRUE(f.is_open());
    f << step;
    EXPECT_TRUE(f.good());
}

TEST(StepWriterTest, WriteToFile_Cone)
{
    constexpr double kPi = 3.14159265358979323846;
    auto body = makeCone(Vec3::zero(), Vec3::unitZ(), kPi / 6.0, 2.0);
    std::string step = StepWriter::write(*body, "Cone");
    std::ofstream f(stepOutputPath("cone.step"));
    ASSERT_TRUE(f.is_open());
    f << step;
    EXPECT_TRUE(f.good());
}

TEST(StepWriterTest, WriteToFile_Torus)
{
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), 2.0, 0.5);
    std::string step = StepWriter::write(*body, "Torus");
    std::ofstream f(stepOutputPath("torus.step"));
    ASSERT_TRUE(f.is_open());
    f << step;
    EXPECT_TRUE(f.good());
}

} // namespace
