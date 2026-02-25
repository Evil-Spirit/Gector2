// Chapter 4.1 — Topology tests: build a unit-cube Body and verify traversal.

#include "gk/brep/BRep.h"
#include <gtest/gtest.h>
#include <unordered_set>

namespace {

using namespace gk;

// ── makeCube() ────────────────────────────────────────────────────────────────
//
// Builds a unit cube [0,1]³ as a Body with:
//   8 Vertices, 12 Edges, 6 Faces (each a quad Wire), 1 Shell, 1 Lump, 1 Body.
//
// Vertices (indices 0-7):
//   v[0]=(0,0,0)  v[1]=(1,0,0)  v[2]=(1,1,0)  v[3]=(0,1,0)
//   v[4]=(0,0,1)  v[5]=(1,0,1)  v[6]=(1,1,1)  v[7]=(0,1,1)
//
// Edges (e[0..11]):
//   Bottom ring: e[0]=(v0,v1), e[1]=(v1,v2), e[2]=(v2,v3), e[3]=(v3,v0)
//   Top ring:    e[4]=(v4,v5), e[5]=(v5,v6), e[6]=(v6,v7), e[7]=(v7,v4)
//   Verticals:   e[8]=(v0,v4), e[9]=(v1,v5), e[10]=(v2,v6), e[11]=(v3,v7)
//
// Each face winds so every wire is closed and each edge appears exactly twice.

static Handle<Body> makeCube()
{
    // ── Vertices ─────────────────────────────────────────────────────────────
    Handle<Vertex> v[8];
    v[0] = makeHandle<Vertex>(Vec3{0,0,0});
    v[1] = makeHandle<Vertex>(Vec3{1,0,0});
    v[2] = makeHandle<Vertex>(Vec3{1,1,0});
    v[3] = makeHandle<Vertex>(Vec3{0,1,0});
    v[4] = makeHandle<Vertex>(Vec3{0,0,1});
    v[5] = makeHandle<Vertex>(Vec3{1,0,1});
    v[6] = makeHandle<Vertex>(Vec3{1,1,1});
    v[7] = makeHandle<Vertex>(Vec3{0,1,1});

    // ── Edges ─────────────────────────────────────────────────────────────────
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

    // Helper: build a closed 4-edge Wire
    auto makeQuadWire = [](Handle<Edge> e0, CoEdgeOrientation o0,
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

    // ── Faces ─────────────────────────────────────────────────────────────────
    // Bottom (z=0): v0→v1→v2→v3→v0
    auto fBottom = makeHandle<Face>();
    fBottom->setOuterWire(makeQuadWire(
        e[0], O::kForward,   // v0→v1
        e[1], O::kForward,   // v1→v2
        e[2], O::kForward,   // v2→v3
        e[3], O::kForward)); // v3→v0

    // Top (z=1): v4→v5→v6→v7→v4
    auto fTop = makeHandle<Face>();
    fTop->setOuterWire(makeQuadWire(
        e[4], O::kForward,   // v4→v5
        e[5], O::kForward,   // v5→v6
        e[6], O::kForward,   // v6→v7
        e[7], O::kForward)); // v7→v4

    // Front (y=0): v0→v1→v5→v4→v0
    auto fFront = makeHandle<Face>();
    fFront->setOuterWire(makeQuadWire(
        e[0],  O::kForward,   // v0→v1
        e[9],  O::kForward,   // v1→v5
        e[4],  O::kReversed,  // v5→v4  (e4=(v4,v5) reversed)
        e[8],  O::kReversed)); // v4→v0  (e8=(v0,v4) reversed)

    // Back (y=1): v2→v3→v7→v6→v2  (e[2] forward, e[11] forward, e[6] rev, e[10] rev)
    auto fBack = makeHandle<Face>();
    fBack->setOuterWire(makeQuadWire(
        e[2],  O::kForward,   // v2→v3
        e[11], O::kForward,   // v3→v7
        e[6],  O::kReversed,  // v7→v6  (e6=(v6,v7) reversed)
        e[10], O::kReversed)); // v6→v2  (e10=(v2,v6) reversed)

    // Left (x=0): v0→v3→v7→v4→v0  (e[3] rev, e[11] fwd, e[7] fwd, e[8] rev)
    auto fLeft = makeHandle<Face>();
    fLeft->setOuterWire(makeQuadWire(
        e[3],  O::kReversed,  // v0→v3  (e3=(v3,v0) reversed)
        e[11], O::kForward,   // v3→v7
        e[7],  O::kForward,   // v7→v4
        e[8],  O::kReversed)); // v4→v0  (e8=(v0,v4) reversed)

    // Right (x=1): v1→v2→v6→v5→v1
    auto fRight = makeHandle<Face>();
    fRight->setOuterWire(makeQuadWire(
        e[1],  O::kForward,   // v1→v2
        e[10], O::kForward,   // v2→v6
        e[5],  O::kReversed,  // v6→v5  (e5=(v5,v6) reversed)
        e[9],  O::kReversed)); // v5→v1  (e9=(v1,v5) reversed)

    // ── Shell ─────────────────────────────────────────────────────────────────
    auto shell = makeHandle<Shell>();
    shell->setClosed(true);
    shell->addFace(fBottom);
    shell->addFace(fTop);
    shell->addFace(fFront);
    shell->addFace(fBack);
    shell->addFace(fLeft);
    shell->addFace(fRight);

    // ── Lump + Body ───────────────────────────────────────────────────────────
    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);

    auto body = makeHandle<Body>();
    body->addLump(lump);
    return body;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST(CubeTopology, FaceCount)
{
    auto body  = makeCube();
    auto shell = body->lumps()[0]->outerShell();
    EXPECT_EQ(shell->numFaces(), static_cast<std::size_t>(6));
}

TEST(CubeTopology, WiresClosed)
{
    auto body = makeCube();
    for (auto& face : body->lumps()[0]->outerShell()->faces()) {
        ASSERT_TRUE(static_cast<bool>(face->outerWire()));
        EXPECT_TRUE(face->outerWire()->isClosed())
            << "A face wire is not closed";
    }
}

TEST(CubeTopology, EdgeCount)
{
    auto body = makeCube();
    std::unordered_set<uint64_t> edgeIds;
    for (auto& face : body->lumps()[0]->outerShell()->faces()) {
        for (auto& ce : face->outerWire()->coEdges()) {
            edgeIds.insert(ce->edge()->id().value());
        }
    }
    EXPECT_EQ(edgeIds.size(), static_cast<std::size_t>(12));
}

TEST(CubeTopology, VertexCount)
{
    auto body = makeCube();
    std::unordered_set<uint64_t> vertIds;
    for (auto& face : body->lumps()[0]->outerShell()->faces()) {
        for (auto& ce : face->outerWire()->coEdges()) {
            auto eg = ce->edge();
            if (eg->start()) vertIds.insert(eg->start()->id().value());
            if (eg->end())   vertIds.insert(eg->end()->id().value());
        }
    }
    EXPECT_EQ(vertIds.size(), static_cast<std::size_t>(8));
}

TEST(CubeTopology, Traversal)
{
    // Verify we can reach all 6 faces from body without crash
    auto body = makeCube();
    int count = 0;
    for (auto& lump : body->lumps())
        for ([[maybe_unused]] auto& face : lump->outerShell()->faces())
            ++count;
    EXPECT_EQ(count, 6);
}

} // namespace
