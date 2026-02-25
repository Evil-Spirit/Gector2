#pragma once
// Chapter 4.4 — BRepIO: ASCII .gkt text serialization of BRep topology.
//
// Format:
//   GECTOR_TEXT_1
//   BODY {
//     LUMP {
//       SHELL outer {
//         VERTEX 1 0.0 0.0 0.0
//         EDGE   100 1 2
//         FACE 200 {
//           WIRE outer {
//             COEDGE 100 forward
//           }
//         }
//       }
//     }
//   }

#include "gk/brep/Body.h"
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <algorithm>

namespace gk {

class BRepIO
{
public:
    // ── Write ─────────────────────────────────────────────────────────────────
    static std::string write(const Body& body)
    {
        std::ostringstream os;
        os << "GECTOR_TEXT_1\n";
        os << "BODY {\n";
        for (auto& lump : body.lumps()) {
            if (!lump) continue;
            os << "  LUMP {\n";
            writeShell(os, lump->outerShell(), "outer", 4);
            for (auto& s : lump->innerShells())
                writeShell(os, s, "inner", 4);
            os << "  }\n";
        }
        os << "}\n";
        return os.str();
    }

    // ── Read ──────────────────────────────────────────────────────────────────
    static Handle<Body> read(const std::string& text)
    {
        // Tokenize: split on whitespace; treat '{' and '}' as separate tokens
        std::vector<std::string> tokens;
        {
            std::string cur;
            for (char ch : text) {
                if (ch == '{' || ch == '}') {
                    if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
                    tokens.push_back(std::string(1, ch));
                } else if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
                    if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
                } else {
                    cur += ch;
                }
            }
            if (!cur.empty()) tokens.push_back(cur);
        }

        auto body = makeHandle<Body>();

        std::unordered_map<int, Handle<Vertex>> vertices;
        std::unordered_map<int, Handle<Edge>>   edges;

        Handle<Lump>  curLump;
        Handle<Shell> curShell;
        Handle<Face>  curFace;
        Handle<Wire>  curWire;

        // Scope stack: 0=none,1=BODY,2=LUMP,3=SHELL,4=FACE,5=WIRE
        std::vector<int> scopeStack;

        std::size_t i = 0;
        while (i < tokens.size()) {
            const std::string& tok = tokens[i];

            if (tok == "GECTOR_TEXT_1") {
                ++i;
            } else if (tok == "BODY") {
                ++i;
                if (i < tokens.size() && tokens[i] == "{") { ++i; scopeStack.push_back(1); }
            } else if (tok == "LUMP") {
                curLump = makeHandle<Lump>();
                ++i;
                if (i < tokens.size() && tokens[i] == "{") { ++i; scopeStack.push_back(2); }
            } else if (tok == "SHELL") {
                curShell = makeHandle<Shell>();
                curShell->setClosed(true);
                if (curLump) curLump->setOuterShell(curShell);
                ++i;
                // skip optional name token (outer / inner)
                if (i < tokens.size() && tokens[i] != "{") ++i;
                if (i < tokens.size() && tokens[i] == "{") { ++i; scopeStack.push_back(3); }
            } else if (tok == "VERTEX") {
                ++i;
                int    id = std::stoi(tokens[i++]);
                double x  = std::stod(tokens[i++]);
                double y  = std::stod(tokens[i++]);
                double z  = std::stod(tokens[i++]);
                vertices[id] = makeHandle<Vertex>(Vec3{x, y, z});
            } else if (tok == "EDGE") {
                ++i;
                int id = std::stoi(tokens[i++]);
                int v1 = std::stoi(tokens[i++]);
                int v2 = std::stoi(tokens[i++]);
                Handle<Vertex> sv = vertices.count(v1) ? vertices.at(v1) : Handle<Vertex>{};
                Handle<Vertex> ev = vertices.count(v2) ? vertices.at(v2) : Handle<Vertex>{};
                edges[id] = makeHandle<Edge>(sv, ev);
            } else if (tok == "FACE") {
                curFace = makeHandle<Face>();
                if (curShell) curShell->addFace(curFace);
                ++i;
                // skip face id token
                if (i < tokens.size() && tokens[i] != "{") ++i;
                if (i < tokens.size() && tokens[i] == "{") { ++i; scopeStack.push_back(4); }
            } else if (tok == "WIRE") {
                curWire = makeHandle<Wire>();
                if (curFace) curFace->setOuterWire(curWire);
                ++i;
                // skip name token (outer / inner)
                if (i < tokens.size() && tokens[i] != "{") ++i;
                if (i < tokens.size() && tokens[i] == "{") { ++i; scopeStack.push_back(5); }
            } else if (tok == "COEDGE") {
                ++i;
                int         eid    = std::stoi(tokens[i++]);
                std::string orient = tokens[i++];
                if (edges.count(eid)) {
                    CoEdgeOrientation o = (orient == "reversed")
                                         ? CoEdgeOrientation::kReversed
                                         : CoEdgeOrientation::kForward;
                    if (curWire) curWire->addCoEdge(makeHandle<CoEdge>(edges.at(eid), o));
                }
            } else if (tok == "}") {
                if (!scopeStack.empty()) {
                    int scope = scopeStack.back();
                    scopeStack.pop_back();
                    if (scope == 2 && curLump) { // LUMP closing
                        body->addLump(curLump);
                        curLump.reset();
                    } else if (scope == 3) {
                        curShell.reset();
                    } else if (scope == 4) {
                        curFace.reset();
                    } else if (scope == 5) {
                        curWire.reset();
                    }
                }
                ++i;
            } else {
                ++i;
            }
        }

        return body;
    }

private:
    static void writeShell(std::ostringstream& os,
                            const Handle<Shell>& shell,
                            const std::string&   name,
                            int                  indent)
    {
        if (!shell) return;
        std::string pad(static_cast<std::size_t>(indent), ' ');

        os << pad << "SHELL " << name << " {\n";

        // Collect vertices and edges with sequential ids
        std::unordered_map<uint64_t, int>             vertIds;
        std::unordered_map<uint64_t, int>             edgeIds;
        std::unordered_map<uint64_t, Handle<Vertex>>  vertMap;
        std::unordered_map<uint64_t, Handle<Edge>>    edgeMap;
        int nextVId = 1;
        int nextEId = 100;

        for (auto& face : shell->faces()) {
            if (!face || !face->outerWire()) continue;
            for (auto& ce : face->outerWire()->coEdges()) {
                if (!ce || !ce->edge()) continue;
                auto e = ce->edge();

                if (e->start()) {
                    uint64_t vid = e->start()->id().value();
                    if (!vertIds.count(vid)) {
                        vertIds[vid] = nextVId++;
                        vertMap[vid] = e->start();
                    }
                }
                if (e->end()) {
                    uint64_t vid = e->end()->id().value();
                    if (!vertIds.count(vid)) {
                        vertIds[vid] = nextVId++;
                        vertMap[vid] = e->end();
                    }
                }
                uint64_t eid = e->id().value();
                if (!edgeIds.count(eid)) {
                    edgeIds[eid] = nextEId++;
                    edgeMap[eid] = e;
                }
            }
        }

        // Write vertices
        for (auto& kv : vertIds) {
            auto& v = vertMap.at(kv.first);
            os << pad << "  VERTEX " << kv.second
               << " " << v->point().x
               << " " << v->point().y
               << " " << v->point().z << "\n";
        }

        // Write edges
        for (auto& kv : edgeIds) {
            auto& e  = edgeMap.at(kv.first);
            int   sv = e->start() ? vertIds.at(e->start()->id().value()) : 0;
            int   ev = e->end()   ? vertIds.at(e->end()->id().value())   : 0;
            os << pad << "  EDGE " << kv.second << " " << sv << " " << ev << "\n";
        }

        // Write faces
        for (auto& face : shell->faces()) {
            if (!face) continue;
            os << pad << "  FACE " << face->id().value() << " {\n";
            if (face->outerWire()) {
                os << pad << "    WIRE outer {\n";
                for (auto& ce : face->outerWire()->coEdges()) {
                    if (!ce || !ce->edge()) continue;
                    int         eid    = edgeIds.at(ce->edge()->id().value());
                    std::string orient = (ce->orientation() == CoEdgeOrientation::kForward)
                                         ? "forward" : "reversed";
                    os << pad << "      COEDGE " << eid << " " << orient << "\n";
                }
                os << pad << "    }\n";
            }
            os << pad << "  }\n";
        }

        os << pad << "}\n";
    }
};

} // namespace gk
