#pragma once
// Chapter 4.1/4.2 — CoEdge: oriented use of an Edge within a Wire.

#include "gk/brep/Edge.h"
#include "gk/curve/ICurve.h"
#include <memory>

namespace gk {

enum class CoEdgeOrientation : uint8_t { kForward, kReversed };

class CoEdge : public TopoEntity
{
public:
    explicit CoEdge(Handle<Edge> e,
                    CoEdgeOrientation o = CoEdgeOrientation::kForward) noexcept
        : TopoEntity(TopoType::kCoEdge)
        , edge_(std::move(e))
        , orientation_(o)
    {}

    Handle<Edge>      edge()        const noexcept { return edge_;        }
    CoEdgeOrientation orientation() const noexcept { return orientation_; }

    void setOrientation(CoEdgeOrientation o) noexcept { orientation_ = o; }

    // ── 2-D pcurve (UV curve on face surface) ────────────────────────────────
    void setPcurve(std::shared_ptr<ICurve2> pc) noexcept { pcurve_ = std::move(pc); }
    const std::shared_ptr<ICurve2>& pcurve() const noexcept { return pcurve_; }

    // ── Vertex accessors respect orientation ─────────────────────────────────
    Handle<Vertex> startVertex() const noexcept
    {
        if (!edge_) return Handle<Vertex>{};
        return (orientation_ == CoEdgeOrientation::kForward)
               ? edge_->start() : edge_->end();
    }

    Handle<Vertex> endVertex() const noexcept
    {
        if (!edge_) return Handle<Vertex>{};
        return (orientation_ == CoEdgeOrientation::kForward)
               ? edge_->end() : edge_->start();
    }

private:
    Handle<Edge>             edge_;
    CoEdgeOrientation        orientation_;
    std::shared_ptr<ICurve2> pcurve_;
};

} // namespace gk
