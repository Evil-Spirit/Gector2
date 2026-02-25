#pragma once
// Chapter 4.1 — Lump: a connected solid region with one outer and zero or more inner shells.

#include "gk/brep/Shell.h"
#include <vector>

namespace gk {

class Lump : public TopoEntity
{
public:
    Lump() noexcept : TopoEntity(TopoType::kLump) {}

    void          setOuterShell(Handle<Shell> s) noexcept { outerShell_ = std::move(s); }
    Handle<Shell> outerShell()                   const noexcept { return outerShell_; }

    void addInnerShell(Handle<Shell> s) { innerShells_.push_back(std::move(s)); }
    const std::vector<Handle<Shell>>& innerShells() const noexcept { return innerShells_; }

private:
    Handle<Shell>              outerShell_;
    std::vector<Handle<Shell>> innerShells_;
};

} // namespace gk
