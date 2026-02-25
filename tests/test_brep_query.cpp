// Chapter 4.3 — BRepQuery tests.

#include "gk/brep/BRep.h"
#include "gk/surface/Plane.h"
#include <gtest/gtest.h>
#include <memory>
#include <cmath>

namespace {

using namespace gk;

// ── Cube with outward-normal Plane surfaces ───────────────────────────────────
//
// The signed-volume formula (1/6)*sign*p0·(p1×p2) accumulates correctly when:
//   * Face orientation sign matches the surface normal direction relative to
//     the outward direction.
//
// Plane normals: uAxis × vAxis.
//
//  Bottom (z=0): Plane((0,0,0),(1,0,0),(0,1,0)) → n=(0,0,+1) inward → kReversed
//  Top    (z=1): Plane((0,0,1),(1,0,0),(0,1,0)) → n=(0,0,+1) outward → kForward
//  Front  (y=0): Plane((0,0,0),(1,0,0),(0,0,1)) → n=(0,-1,0) outward → kForward
//  Back   (y=1): Plane((1,1,0),(-1,0,0),(0,0,1)) → n=(0,+1,0) outward → kForward
//  Left   (x=0): Plane((0,0,0),(0,0,1),(0,1,0)) → n=(-1,0,0) outward → kForward
//  Right  (x=1): Plane((1,0,0),(0,1,0),(0,0,1)) → n=(+1,0,0) outward → kForward

static Handle<Body> makeCubeWithSurfaces()
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

    SurfaceDomain dom01{Interval{0,1}, Interval{0,1}};

    // Bottom
    auto fBottom = makeHandle<Face>();
    fBottom->setOuterWire(makeQuad(e[0],O::kForward, e[1],O::kForward,
                                    e[2],O::kForward, e[3],O::kForward));
    fBottom->setSurface(std::make_shared<Plane>(Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,1,0}));
    fBottom->setOrientation(FaceOrientation::kReversed);
    fBottom->setUVDomain(dom01);

    // Top
    auto fTop = makeHandle<Face>();
    fTop->setOuterWire(makeQuad(e[4],O::kForward, e[5],O::kForward,
                                 e[6],O::kForward, e[7],O::kForward));
    fTop->setSurface(std::make_shared<Plane>(Vec3{0,0,1}, Vec3{1,0,0}, Vec3{0,1,0}));
    fTop->setOrientation(FaceOrientation::kForward);
    fTop->setUVDomain(dom01);

    // Front
    auto fFront = makeHandle<Face>();
    fFront->setOuterWire(makeQuad(e[0],O::kForward,  e[9],O::kForward,
                                   e[4],O::kReversed, e[8],O::kReversed));
    fFront->setSurface(std::make_shared<Plane>(Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,0,1}));
    fFront->setOrientation(FaceOrientation::kForward);
    fFront->setUVDomain(dom01);

    // Back
    auto fBack = makeHandle<Face>();
    fBack->setOuterWire(makeQuad(e[2],O::kForward,  e[11],O::kForward,
                                  e[6],O::kReversed, e[10],O::kReversed));
    fBack->setSurface(std::make_shared<Plane>(Vec3{1,1,0}, Vec3{-1,0,0}, Vec3{0,0,1}));
    fBack->setOrientation(FaceOrientation::kForward);
    fBack->setUVDomain(dom01);

    // Left
    auto fLeft = makeHandle<Face>();
    fLeft->setOuterWire(makeQuad(e[3],O::kReversed, e[11],O::kForward,
                                  e[7],O::kForward,  e[8],O::kReversed));
    fLeft->setSurface(std::make_shared<Plane>(Vec3{0,0,0}, Vec3{0,0,1}, Vec3{0,1,0}));
    fLeft->setOrientation(FaceOrientation::kForward);
    fLeft->setUVDomain(dom01);

    // Right
    auto fRight = makeHandle<Face>();
    fRight->setOuterWire(makeQuad(e[1],O::kForward,  e[10],O::kForward,
                                   e[5],O::kReversed, e[9],O::kReversed));
    fRight->setSurface(std::make_shared<Plane>(Vec3{1,0,0}, Vec3{0,1,0}, Vec3{0,0,1}));
    fRight->setOrientation(FaceOrientation::kForward);
    fRight->setUVDomain(dom01);

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

TEST(BRepQueryTest, CubeSurfaceArea)
{
    auto body = makeCubeWithSurfaces();
    auto mp   = BRepQuery::computeMassProperties(*body, 16);
    EXPECT_NEAR(mp.surfaceArea, 6.0, 1e-4);
}

TEST(BRepQueryTest, CubeVolume)
{
    auto body = makeCubeWithSurfaces();
    auto mp   = BRepQuery::computeMassProperties(*body, 16);
    EXPECT_NEAR(mp.volume, 1.0, 1e-4);
}

TEST(BRepQueryTest, PointInsideCube)
{
    auto body = makeCubeWithSurfaces();
    auto loc  = BRepQuery::classifyPoint(*body, Vec3{0.5, 0.5, 0.5});
    EXPECT_EQ(loc, PointLocation::kInside);
}

TEST(BRepQueryTest, PointOutsideCube)
{
    auto body = makeCubeWithSurfaces();
    auto loc  = BRepQuery::classifyPoint(*body, Vec3{2.0, 0.5, 0.5});
    EXPECT_EQ(loc, PointLocation::kOutside);
}

TEST(BRepQueryTest, TopologicalDistance)
{
    auto body  = makeCubeWithSurfaces();
    auto shell = body->lumps()[0]->outerShell();
    const auto& faces = shell->faces();

    // Bottom (index 0) and Front (index 2) share edge e[0] → distance = 1
    int d = BRepQuery::topologicalDistance(*faces[0], *faces[2], *shell);
    EXPECT_EQ(d, 1);
}

} // namespace
