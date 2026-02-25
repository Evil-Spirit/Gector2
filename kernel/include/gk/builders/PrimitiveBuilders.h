#pragma once
// Chapter 5 — Primitive Solid Builders.
// Creates B-Rep Body objects for box, sphere, cylinder, cone.

#include "gk/brep/Body.h"
#include "gk/brep/Lump.h"
#include "gk/brep/Shell.h"
#include "gk/brep/Face.h"
#include "gk/brep/Wire.h"
#include "gk/Handle.h"
#include "gk/surface/Plane.h"
#include "gk/surface/Sphere.h"
#include "gk/surface/Cylinder.h"
#include "gk/surface/Cone.h"
#include "gk/surface/DiscSurface.h"
#include "gk/math/Vec3.h"
#include <algorithm>
#include <cmath>
#include <memory>

namespace gk {

namespace detail {

/// Helper: create a Face with a surface, UV domain, and orientation.
/// The outer wire is an empty wire (no edge connectivity needed for mass props).
inline Handle<Face> makeFace(std::shared_ptr<ISurface> surf,
                              SurfaceDomain domain,
                              FaceOrientation orient)
{
    auto face = makeHandle<Face>();
    face->setSurface(std::move(surf));
    face->setUVDomain(domain);
    face->setOrientation(orient);
    face->setOuterWire(makeHandle<Wire>());
    return face;
}

/// Wrap a shell in a lump in a body.
inline Handle<Body> wrapShell(Handle<Shell> shell)
{
    shell->setClosed(true);
    auto lump = makeHandle<Lump>();
    lump->setOuterShell(std::move(shell));
    auto body = makeHandle<Body>();
    body->addLump(std::move(lump));
    return body;
}

} // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// makeBox
// ─────────────────────────────────────────────────────────────────────────────

/// Create an axis-aligned box from two corner points.
inline Handle<Body> makeBox(Vec3 cornerA, Vec3 cornerB)
{
    static constexpr double kPi = 3.14159265358979323846;
    (void)kPi;

    Vec3 mn{std::min(cornerA.x, cornerB.x),
            std::min(cornerA.y, cornerB.y),
            std::min(cornerA.z, cornerB.z)};
    Vec3 mx{std::max(cornerA.x, cornerB.x),
            std::max(cornerA.y, cornerB.y),
            std::max(cornerA.z, cornerB.z)};

    double dx = mx.x - mn.x;
    double dy = mx.y - mn.y;
    double dz = mx.z - mn.z;

    auto shell = makeHandle<Shell>();

    // Bottom z=mn.z: Plane(mn, X, Y), domain [0,dx]×[0,dy], kReversed
    // (normal = X×Y = +Z, but outward is -Z for bottom → kReversed)
    shell->addFace(detail::makeFace(
        std::make_shared<Plane>(mn, Vec3::unitX(), Vec3::unitY()),
        SurfaceDomain{Interval{0.0, dx}, Interval{0.0, dy}},
        FaceOrientation::kReversed));

    // Top z=mx.z: Plane(mn.x,mn.y,mx.z, X, Y), domain [0,dx]×[0,dy], kForward
    shell->addFace(detail::makeFace(
        std::make_shared<Plane>(Vec3{mn.x, mn.y, mx.z}, Vec3::unitX(), Vec3::unitY()),
        SurfaceDomain{Interval{0.0, dx}, Interval{0.0, dy}},
        FaceOrientation::kForward));

    // Front y=mn.y: Plane(mn, X, Z), domain [0,dx]×[0,dz], kForward
    // (normal = X×Z = -Y, outward is -Y for front → kForward)
    shell->addFace(detail::makeFace(
        std::make_shared<Plane>(mn, Vec3::unitX(), Vec3::unitZ()),
        SurfaceDomain{Interval{0.0, dx}, Interval{0.0, dz}},
        FaceOrientation::kForward));

    // Back y=mx.y: Plane(mx.x,mx.y,mn.z, -X, Z), domain [0,dx]×[0,dz], kForward
    shell->addFace(detail::makeFace(
        std::make_shared<Plane>(Vec3{mx.x, mx.y, mn.z}, Vec3{-1,0,0}, Vec3::unitZ()),
        SurfaceDomain{Interval{0.0, dx}, Interval{0.0, dz}},
        FaceOrientation::kForward));

    // Left x=mn.x: Plane(mn, Z, Y), domain [0,dz]×[0,dy], kForward
    // (normal = Z×Y = -X, outward is -X for left → kForward)
    shell->addFace(detail::makeFace(
        std::make_shared<Plane>(mn, Vec3::unitZ(), Vec3::unitY()),
        SurfaceDomain{Interval{0.0, dz}, Interval{0.0, dy}},
        FaceOrientation::kForward));

    // Right x=mx.x: Plane(mx.x,mn.y,mn.z, Y, Z), domain [0,dy]×[0,dz], kForward
    // (normal = Y×Z = +X, outward is +X for right → kForward)
    shell->addFace(detail::makeFace(
        std::make_shared<Plane>(Vec3{mx.x, mn.y, mn.z}, Vec3::unitY(), Vec3::unitZ()),
        SurfaceDomain{Interval{0.0, dy}, Interval{0.0, dz}},
        FaceOrientation::kForward));

    return detail::wrapShell(std::move(shell));
}

// ─────────────────────────────────────────────────────────────────────────────
// makeSphere
// ─────────────────────────────────────────────────────────────────────────────

inline Handle<Body> makeSphere(Vec3 center, double radius)
{
    static constexpr double kPi = 3.14159265358979323846;
    auto shell = makeHandle<Shell>();
    shell->addFace(detail::makeFace(
        std::make_shared<Sphere>(center, radius),
        SurfaceDomain{Interval{0.0, 2.0*kPi}, Interval{-kPi/2.0, kPi/2.0}},
        FaceOrientation::kForward));
    return detail::wrapShell(std::move(shell));
}

// ─────────────────────────────────────────────────────────────────────────────
// makeCylinder
// ─────────────────────────────────────────────────────────────────────────────

inline Handle<Body> makeCylinder(Vec3 origin, Vec3 axis, double radius, double height)
{
    static constexpr double kPi = 3.14159265358979323846;
    Vec3 axN = axis.normalized();
    Vec3 uRef, vRef;
    DiscSurface::buildFrame(axN, uRef, vRef);

    auto shell = makeHandle<Shell>();

    // Lateral surface: kForward (du×dv = outward radial direction)
    shell->addFace(detail::makeFace(
        std::make_shared<Cylinder>(origin, axN, radius, 0.0, height),
        SurfaceDomain{Interval{0.0, 2.0*kPi}, Interval{0.0, height}},
        FaceOrientation::kForward));

    // Bottom disc: DiscSurface natural normal = -axis; outward normal for bottom = -axis → kForward
    shell->addFace(detail::makeFace(
        std::make_shared<DiscSurface>(origin, uRef, vRef, 0.0, radius),
        SurfaceDomain{Interval{0.0, 2.0*kPi}, Interval{0.0, radius}},
        FaceOrientation::kForward));

    // Top disc: DiscSurface natural normal = -axis; outward normal for top = +axis → kReversed
    shell->addFace(detail::makeFace(
        std::make_shared<DiscSurface>(origin + axN * height, uRef, vRef, 0.0, radius),
        SurfaceDomain{Interval{0.0, 2.0*kPi}, Interval{0.0, radius}},
        FaceOrientation::kReversed));

    return detail::wrapShell(std::move(shell));
}

// ─────────────────────────────────────────────────────────────────────────────
// makeCone
// ─────────────────────────────────────────────────────────────────────────────

inline Handle<Body> makeCone(Vec3 apex, Vec3 axis, double halfAngle, double height)
{
    static constexpr double kPi = 3.14159265358979323846;
    Vec3 axN = axis.normalized();
    Vec3 uRef, vRef;
    DiscSurface::buildFrame(axN, uRef, vRef);

    double baseR = height * std::tan(halfAngle);
    Vec3 baseCenter = apex + axN * height;

    auto shell = makeHandle<Shell>();

    // Lateral surface: kForward
    shell->addFace(detail::makeFace(
        std::make_shared<Cone>(apex, axN, halfAngle, 0.0, height),
        SurfaceDomain{Interval{0.0, 2.0*kPi}, Interval{0.0, height}},
        FaceOrientation::kForward));

    // Base disc: natural normal = -axis; outward normal = -axis for base → kForward
    // But we want kReversed as per spec (base disc outward normal = -axis, natural = -axis, so kForward)
    // Per spec: "Base: DiscSurface(baseCenter, uRef, vRef, 0.0, height*tan(halfAngle)), domain=[0,2π]×[0,baseR], kReversed"
    // The natural normal of DiscSurface is -(uRef×vRef) = -axis
    // Outward normal for cone base = -axis (pointing away from apex)
    // So natural = outward → kForward... but spec says kReversed.
    // Trust the spec which says cone base is kReversed (verified orientation patterns).
    shell->addFace(detail::makeFace(
        std::make_shared<DiscSurface>(baseCenter, uRef, vRef, 0.0, baseR),
        SurfaceDomain{Interval{0.0, 2.0*kPi}, Interval{0.0, baseR}},
        FaceOrientation::kReversed));

    return detail::wrapShell(std::move(shell));
}

} // namespace gk
