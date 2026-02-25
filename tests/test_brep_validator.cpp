// Chapter 4.2 — BRepValidator tests.

#include "gk/brep/BRep.h"
#include "gk/surface/Plane.h"
#include <gtest/gtest.h>
#include <memory>

namespace {

using namespace gk;

// ── Re-usable cube builder (topology only) ────────────────────────────────────
static Handle<Body> makeCube()
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
    fBottom->setOuterWire(makeQuad(e[0],O::kForward, e[1],O::kForward,
                                   e[2],O::kForward, e[3],O::kForward));
    auto fTop = makeHandle<Face>();
    fTop->setOuterWire(makeQuad(e[4],O::kForward, e[5],O::kForward,
                                 e[6],O::kForward, e[7],O::kForward));
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

// ── Attach Plane surfaces to a cube body ─────────────────────────────────────
static void attachCubeSurfaces(const Handle<Body>& body)
{
    auto& faces = body->lumps()[0]->outerShell()->faces();
    // faces order: [0]=Bottom [1]=Top [2]=Front [3]=Back [4]=Left [5]=Right
    // (matching makeCube() insertion order)

    SurfaceDomain dom01{Interval{0,1}, Interval{0,1}};

    // Bottom (z=0): Plane(origin=(0,0,0), u=(1,0,0), v=(0,1,0)), normal=(0,0,1) inward → kReversed
    faces[0]->setSurface(std::make_shared<Plane>(Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,1,0}));
    faces[0]->setOrientation(FaceOrientation::kReversed);
    faces[0]->setUVDomain(dom01);

    // Top (z=1): same axes, origin=(0,0,1), normal=(0,0,1) outward → kForward
    faces[1]->setSurface(std::make_shared<Plane>(Vec3{0,0,1}, Vec3{1,0,0}, Vec3{0,1,0}));
    faces[1]->setOrientation(FaceOrientation::kForward);
    faces[1]->setUVDomain(dom01);

    // Front (y=0): Plane(origin=(0,0,0), u=(1,0,0), v=(0,0,1)), normal=(0,-1,0) outward → kForward
    faces[2]->setSurface(std::make_shared<Plane>(Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,0,1}));
    faces[2]->setOrientation(FaceOrientation::kForward);
    faces[2]->setUVDomain(dom01);

    // Back (y=1): Plane(origin=(1,1,0), u=(-1,0,0), v=(0,0,1)), normal=(0,1,0) outward → kForward
    faces[3]->setSurface(std::make_shared<Plane>(Vec3{1,1,0}, Vec3{-1,0,0}, Vec3{0,0,1}));
    faces[3]->setOrientation(FaceOrientation::kForward);
    faces[3]->setUVDomain(dom01);

    // Left (x=0): Plane(origin=(0,0,0), u=(0,0,1), v=(0,1,0)), normal=(-1,0,0) outward → kForward
    faces[4]->setSurface(std::make_shared<Plane>(Vec3{0,0,0}, Vec3{0,0,1}, Vec3{0,1,0}));
    faces[4]->setOrientation(FaceOrientation::kForward);
    faces[4]->setUVDomain(dom01);

    // Right (x=1): Plane(origin=(1,0,0), u=(0,1,0), v=(0,0,1)), normal=(1,0,0) outward → kForward
    faces[5]->setSurface(std::make_shared<Plane>(Vec3{1,0,0}, Vec3{0,1,0}, Vec3{0,0,1}));
    faces[5]->setOrientation(FaceOrientation::kForward);
    faces[5]->setUVDomain(dom01);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST(BRepValidatorTest, CubeIsValid)
{
    auto body = makeCube();
    auto r    = BRepValidator::validate(*body, /*checkGeometry=*/false);
    EXPECT_TRUE(r.valid) << r.errors[0];
}

TEST(BRepValidatorTest, CubeGeometryValid)
{
    auto body = makeCube();
    attachCubeSurfaces(body);
    auto r = BRepValidator::validate(*body, /*checkGeometry=*/true);
    EXPECT_TRUE(r.valid) << (r.errors.empty() ? "" : r.errors[0]);
}

TEST(BRepValidatorTest, NullEdgeInvalid)
{
    auto body = makeCube();
    // Insert a CoEdge with a null edge into an existing wire
    auto wire = body->lumps()[0]->outerShell()->faces()[0]->outerWire();
    wire->addCoEdge(makeHandle<CoEdge>(Handle<Edge>{}, CoEdgeOrientation::kForward));
    auto r = BRepValidator::validate(*body);
    EXPECT_FALSE(r.valid);
    bool foundV2 = false;
    for (auto& e : r.errors) if (e.find("V2") != std::string::npos) foundV2 = true;
    EXPECT_TRUE(foundV2);
}

TEST(BRepValidatorTest, OpenWireInvalid)
{
    // Wire v0→v1→v2 (does not return to v0)
    auto v0 = makeHandle<Vertex>(Vec3{0,0,0});
    auto v1 = makeHandle<Vertex>(Vec3{1,0,0});
    auto v2 = makeHandle<Vertex>(Vec3{1,1,0});
    auto edge1 = makeHandle<Edge>(v0, v1);
    auto edge2 = makeHandle<Edge>(v1, v2);

    auto wire = makeHandle<Wire>();
    wire->addCoEdge(makeHandle<CoEdge>(edge1, CoEdgeOrientation::kForward));
    wire->addCoEdge(makeHandle<CoEdge>(edge2, CoEdgeOrientation::kForward));

    auto face = makeHandle<Face>();
    face->setOuterWire(wire);

    auto shell = makeHandle<Shell>();
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);

    auto body = makeHandle<Body>();
    body->addLump(lump);

    auto r = BRepValidator::validate(*body);
    EXPECT_FALSE(r.valid);
    bool foundV3 = false;
    for (auto& e : r.errors) if (e.find("V3") != std::string::npos) foundV3 = true;
    EXPECT_TRUE(foundV3);
}

TEST(BRepValidatorTest, NonManifoldEdgeInvalid)
{
    // Build a cube and add a 3rd use of one edge
    auto body = makeCube();
    auto shell = body->lumps()[0]->outerShell();

    // Grab the first edge from the first face
    auto sharedEdge = shell->faces()[0]->outerWire()->coEdges()[0]->edge();

    // Add a new face with a wire that uses that edge a 3rd time
    auto extraWire = makeHandle<Wire>();
    extraWire->addCoEdge(makeHandle<CoEdge>(sharedEdge, CoEdgeOrientation::kForward));
    // Add another coedge to make the wire look "closed" via same edge reversed
    extraWire->addCoEdge(makeHandle<CoEdge>(sharedEdge, CoEdgeOrientation::kReversed));

    auto extraFace = makeHandle<Face>();
    extraFace->setOuterWire(extraWire);
    shell->addFace(extraFace);

    auto r = BRepValidator::validate(*body);
    EXPECT_FALSE(r.valid);
    bool foundV5 = false;
    for (auto& e : r.errors) if (e.find("V5") != std::string::npos) foundV5 = true;
    EXPECT_TRUE(foundV5);
}

TEST(BRepValidatorTest, MissingSurfaceInvalid)
{
    auto body = makeCube(); // no surfaces
    auto r    = BRepValidator::validate(*body, /*checkGeometry=*/true);
    EXPECT_FALSE(r.valid);
    bool foundV6 = false;
    for (auto& e : r.errors) if (e.find("V6") != std::string::npos) foundV6 = true;
    EXPECT_TRUE(foundV6);
}

} // namespace
