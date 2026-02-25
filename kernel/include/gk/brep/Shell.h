#pragma once
// Chapter 4.1 — Shell: collection of Faces forming a connected surface.

#include "gk/brep/Face.h"
#include <vector>

namespace gk {

class Shell : public TopoEntity
{
public:
    Shell() noexcept : TopoEntity(TopoType::kShell) {}

    void addFace(Handle<Face> f) { faces_.push_back(std::move(f)); }

    const std::vector<Handle<Face>>& faces()    const noexcept { return faces_; }
    std::size_t                      numFaces()  const noexcept { return faces_.size(); }

    bool isClosed()             const noexcept { return closed_; }
    void setClosed(bool c = true) noexcept { closed_ = c; }

private:
    std::vector<Handle<Face>> faces_;
    bool                      closed_{false};
};

} // namespace gk
