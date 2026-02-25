#pragma once
// Chapter 4.2 — BRepValidator: topology and geometry consistency checks.

#include "gk/brep/Body.h"
#include <string>
#include <vector>
#include <unordered_map>

namespace gk {

struct ValidationResult
{
    bool                     valid{true};
    std::vector<std::string> errors;
};

class BRepValidator
{
public:
    static ValidationResult validate(const Body& body, bool checkGeometry = false)
    {
        ValidationResult r;
        for (auto& lump : body.lumps()) {
            if (!lump) continue;
            validateShell(lump->outerShell(), r, checkGeometry);
            for (auto& s : lump->innerShells())
                validateShell(s, r, checkGeometry);
        }
        return r;
    }

private:
    static void addError(ValidationResult& r, const std::string& msg)
    {
        r.valid = false;
        r.errors.push_back(msg);
    }

    static void validateShell(const Handle<Shell>& shell,
                               ValidationResult&    r,
                               bool                 checkGeom)
    {
        if (!shell) return;

        // For V5 manifold check
        std::unordered_map<uint64_t, int> edgeCount;

        for (auto& face : shell->faces()) {
            if (!face) continue;

            // V4: face must have outer wire
            if (!face->outerWire())
                addError(r, "V4: Face missing outer wire");

            // V6 (geometry): face must have surface
            if (checkGeom && !face->hasSurface())
                addError(r, "V6: Face missing surface");

            // Validate each wire
            auto validateWire = [&](const Handle<Wire>& wire) {
                if (!wire) return;

                // V3: wire must be closed
                if (!wire->isClosed())
                    addError(r, "V3: Wire is not closed");

                for (auto& ce : wire->coEdges()) {
                    if (!ce) continue;

                    // V2: coedge must reference a non-null edge
                    if (!ce->edge()) {
                        addError(r, "V2: CoEdge has null edge");
                        continue;
                    }

                    // V1: edge must have non-null start and end vertices
                    if (!ce->edge()->start() || !ce->edge()->end())
                        addError(r, "V1: Edge has null vertex");

                    edgeCount[ce->edge()->id().value()]++;
                }
            };

            if (face->outerWire()) validateWire(face->outerWire());
            for (auto& iw : face->innerWires()) validateWire(iw);
        }

        // V5 manifold check: in a closed shell each edge appears exactly twice
        if (shell->isClosed()) {
            for (auto& kv : edgeCount) {
                if (kv.second != 2)
                    addError(r, "V5: Edge appears " + std::to_string(kv.second) +
                                " times (manifold requires exactly 2)");
            }
        }
    }
};

} // namespace gk
