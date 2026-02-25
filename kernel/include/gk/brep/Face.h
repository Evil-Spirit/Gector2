#pragma once
// Chapter 4.1/4.2 — Face: a bounded region of a surface.

#include "gk/brep/Wire.h"
#include "gk/surface/ISurface.h"
#include <memory>
#include <vector>

namespace gk {

enum class FaceOrientation : uint8_t { kForward, kReversed };

class Face : public TopoEntity
{
public:
    Face() noexcept : TopoEntity(TopoType::kFace) {}

    // ── Wires ────────────────────────────────────────────────────────────────
    Handle<Wire>       outerWire()  const noexcept { return outerWire_; }
    void               setOuterWire(Handle<Wire> w) noexcept { outerWire_ = std::move(w); }

    void               addInnerWire(Handle<Wire> w) { innerWires_.push_back(std::move(w)); }
    const std::vector<Handle<Wire>>& innerWires() const noexcept { return innerWires_; }

    // ── Orientation ──────────────────────────────────────────────────────────
    FaceOrientation orientation()             const noexcept { return orientation_; }
    void            setOrientation(FaceOrientation o) noexcept { orientation_ = o; }

    // ── Geometric binding ────────────────────────────────────────────────────
    void setSurface(std::shared_ptr<ISurface> s) noexcept { surface_ = std::move(s); }
    const std::shared_ptr<ISurface>& surface()  const noexcept { return surface_; }
    bool hasSurface() const noexcept { return surface_ != nullptr; }

    void          setUVDomain(const SurfaceDomain& d) noexcept { uvDomain_ = d; uvDomainSet_ = true; }
    bool          hasUVDomain() const noexcept { return uvDomainSet_; }

    /// Returns the set domain if one was set; otherwise the surface's natural domain().
    SurfaceDomain uvDomain() const noexcept
    {
        if (uvDomainSet_) return uvDomain_;
        if (surface_)     return surface_->domain();
        return SurfaceDomain{};
    }

private:
    Handle<Wire>              outerWire_;
    std::vector<Handle<Wire>> innerWires_;
    FaceOrientation           orientation_{FaceOrientation::kForward};
    std::shared_ptr<ISurface> surface_;
    SurfaceDomain             uvDomain_{};
    bool                      uvDomainSet_{false};
};

} // namespace gk
