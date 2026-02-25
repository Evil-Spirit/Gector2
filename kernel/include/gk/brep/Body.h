#pragma once
// Chapter 4.1 — Body: top-level solid model container.

#include "gk/brep/Lump.h"
#include <vector>

namespace gk {

class Body : public TopoEntity
{
public:
    Body() noexcept : TopoEntity(TopoType::kBody) {}

    void addLump(Handle<Lump> l) { lumps_.push_back(std::move(l)); }

    const std::vector<Handle<Lump>>& lumps()    const noexcept { return lumps_; }
    std::size_t                      numLumps()  const noexcept { return lumps_.size(); }
    bool                             isEmpty()   const noexcept { return lumps_.empty(); }

private:
    std::vector<Handle<Lump>> lumps_;
};

} // namespace gk
