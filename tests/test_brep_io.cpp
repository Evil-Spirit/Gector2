// Chapter 4.4 — BRepIO tests.

#include "gk/brep/BRep.h"
#include <gtest/gtest.h>
#include <unordered_set>
#include <string>

namespace {

using namespace gk;

// ── Minimal cube builder (topology only) ─────────────────────────────────────
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

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST(BRepIOTest, TextFormatNotEmpty)
{
    auto   body = makeCube();
    std::string text = BRepIO::write(*body);
    EXPECT_FALSE(text.empty());
    EXPECT_NE(text.find("BODY"), std::string::npos);
}

TEST(BRepIOTest, WriteAndRead)
{
    auto original = makeCube();
    std::string text = BRepIO::write(*original);

    auto readBack = BRepIO::read(text);
    ASSERT_TRUE(readBack && !readBack->isEmpty());

    // Count vertices, edges, faces in read-back body
    std::unordered_set<uint64_t> vertIds, edgeIds;
    int faceCount = 0;

    for (auto& lump : readBack->lumps()) {
        if (!lump || !lump->outerShell()) continue;
        for (auto& face : lump->outerShell()->faces()) {
            ++faceCount;
            if (!face->outerWire()) continue;
            for (auto& ce : face->outerWire()->coEdges()) {
                if (!ce || !ce->edge()) continue;
                edgeIds.insert(ce->edge()->id().value());
                if (ce->edge()->start()) vertIds.insert(ce->edge()->start()->id().value());
                if (ce->edge()->end())   vertIds.insert(ce->edge()->end()->id().value());
            }
        }
    }

    EXPECT_EQ(faceCount, 6);
    EXPECT_EQ(edgeIds.size(), static_cast<std::size_t>(12));
    EXPECT_EQ(vertIds.size(), static_cast<std::size_t>(8));
}

} // namespace
