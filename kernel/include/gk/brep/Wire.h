#pragma once
// Chapter 4.1 — Wire: ordered sequence of CoEdges forming a closed loop.

#include "gk/brep/CoEdge.h"
#include <vector>

namespace gk {

class Wire : public TopoEntity
{
public:
    Wire() noexcept : TopoEntity(TopoType::kWire) {}

    void addCoEdge(Handle<CoEdge> ce) { coEdges_.push_back(std::move(ce)); }

    const std::vector<Handle<CoEdge>>& coEdges()    const noexcept { return coEdges_; }
    std::size_t                        numCoEdges()  const noexcept { return coEdges_.size(); }

    /// True if the last coedge's endVertex == first coedge's startVertex.
    bool isClosed() const noexcept
    {
        if (coEdges_.empty()) return false;
        auto sv = coEdges_.front()->startVertex();
        auto ev = coEdges_.back()->endVertex();
        if (!sv || !ev) return false;
        return sv->id() == ev->id();
    }

private:
    std::vector<Handle<CoEdge>> coEdges_;
};

} // namespace gk
