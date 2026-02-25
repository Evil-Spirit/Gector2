#pragma once
// Chapter 4.1/4.2 — Edge: topological edge with optional geometric binding.

#include "gk/brep/Vertex.h"
#include "gk/curve/ICurve.h"
#include <memory>
#include <cmath>

namespace gk {

class Edge : public TopoEntity
{
public:
    Edge(Handle<Vertex> s, Handle<Vertex> e) noexcept
        : TopoEntity(TopoType::kEdge)
        , start_(std::move(s))
        , end_(std::move(e))
    {}

    Handle<Vertex> start() const noexcept { return start_; }
    Handle<Vertex> end()   const noexcept { return end_;   }

    void setStart(Handle<Vertex> v) noexcept { start_ = std::move(v); }
    void setEnd  (Handle<Vertex> v) noexcept { end_   = std::move(v); }

    // ── Geometric binding ────────────────────────────────────────────────────
    void setCurve(std::shared_ptr<ICurve3> c, double tStart, double tEnd) noexcept
    {
        curve_  = std::move(c);
        tStart_ = tStart;
        tEnd_   = tEnd;
    }

    const std::shared_ptr<ICurve3>& curve()   const noexcept { return curve_; }
    bool                            hasCurve() const noexcept { return curve_ != nullptr; }
    double                          tStart()   const noexcept { return tStart_; }
    double                          tEnd()     const noexcept { return tEnd_;   }

    /// Approximate arc length: 64-sample polyline or straight-line if no curve.
    double length() const
    {
        if (hasCurve()) {
            constexpr int kN = 64;
            double len = 0.0;
            Vec3 prev = curve_->evaluate(tStart_).p;
            for (int i = 1; i <= kN; ++i) {
                double t = tStart_ + (tEnd_ - tStart_) * (double(i) / double(kN));
                Vec3   cur = curve_->evaluate(t).p;
                len += (cur - prev).norm();
                prev = cur;
            }
            return len;
        }
        if (start_ && end_)
            return (end_->point() - start_->point()).norm();
        return 0.0;
    }

private:
    Handle<Vertex>           start_;
    Handle<Vertex>           end_;
    std::shared_ptr<ICurve3> curve_;
    double                   tStart_{0.0};
    double                   tEnd_  {1.0};
};

} // namespace gk
