#pragma once
// Chapter 4.1 — Topological base entity types.

#include "gk/Handle.h"
#include "gk/EntityId.h"
#include <cstdint>

namespace gk {

enum class TopoType : uint8_t {
    kVertex, kEdge, kCoEdge, kWire, kFace, kShell, kLump, kBody
};

class TopoEntity : public RefCounted
{
public:
    TopoType type() const noexcept { return type_; }
    EntityId id()   const noexcept { return id_;   }

protected:
    explicit TopoEntity(TopoType t) noexcept
        : type_(t), id_(EntityId::generate()) {}

private:
    TopoType type_;
    EntityId id_;
};

} // namespace gk
