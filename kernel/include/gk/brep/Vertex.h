#pragma once
// Chapter 4.1 — Vertex: a topological vertex with a 3-D point.

#include "gk/brep/TopoTypes.h"
#include "gk/math/Vec3.h"

namespace gk {

class Vertex : public TopoEntity
{
public:
    explicit Vertex(const Vec3& pt) noexcept
        : TopoEntity(TopoType::kVertex), point_(pt) {}

    const Vec3& point() const noexcept { return point_; }
    void        setPoint(const Vec3& pt) noexcept { point_ = pt; }

private:
    Vec3 point_;
};

} // namespace gk
