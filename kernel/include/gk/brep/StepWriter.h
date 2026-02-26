#pragma once
// Chapter 9.1 — STEP AP203/AP214 Export.
//
// Exports a gk::Body to an ISO 10303-21 STEP file string.
//
// Supported geometry:
//   Surfaces: Plane, Sphere, Cylinder, Cone, Torus, DiscSurface,
//             BSplineSurface, NURBSSurface.
//   Curves:   Line3, Circle3, BSplineCurve3, NURBSCurve3.
//   Topology: Vertex, Edge (with or without curve binding), CoEdge, Wire,
//             Face (with or without surface), Shell, Lump, Body.
//
// Usage:
//   std::string step = gk::StepWriter::write(body);

#include "gk/brep/Body.h"
#include "gk/brep/CoEdge.h"
#include "gk/surface/BSplineSurface.h"
#include "gk/surface/Cone.h"
#include "gk/surface/Cylinder.h"
#include "gk/surface/DiscSurface.h"
#include "gk/surface/NURBSSurface.h"
#include "gk/surface/Plane.h"
#include "gk/surface/Sphere.h"
#include "gk/surface/Torus.h"
#include "gk/curve/BSplineCurve.h"
#include "gk/curve/Circle.h"
#include "gk/curve/Ellipse.h"
#include "gk/curve/Line.h"
#include "gk/curve/NURBSCurve.h"

#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace gk {

class StepWriter
{
public:
    /// Export @p body as an ISO 10303-21 STEP AP214 string.
    /// @param name  Optional part name embedded in the STEP header.
    static std::string write(const Body& body,
                             const std::string& name = "GectorPart")
    {
        StepWriter sw;
        sw.buildEntities(body);
        return sw.generate(name);
    }

private:
    // ── Entity storage ────────────────────────────────────────────────────────

    struct Entity
    {
        int         id;
        std::string text; // entity text without leading "#id=" and trailing ";"
    };

    std::vector<Entity> entities_;
    int                 nextId_{1};

    int newId() { return nextId_++; }

    int emit(std::string text)
    {
        int id = newId();
        entities_.push_back({id, std::move(text)});
        return id;
    }

    // ── Format helpers ────────────────────────────────────────────────────────

    static std::string fmtReal(double v)
    {
        std::ostringstream ss;
        ss.precision(15);
        ss << v;
        std::string s = ss.str();
        // Ensure STEP-required decimal point
        if (s.find('.') == std::string::npos &&
            s.find('e') == std::string::npos &&
            s.find('E') == std::string::npos)
            s += ".";
        return s;
    }

    static std::string ref(int id)
    {
        return "#" + std::to_string(id);
    }

    static std::string refList(const std::vector<int>& ids)
    {
        std::string s = "(";
        for (std::size_t i = 0; i < ids.size(); ++i) {
            if (i > 0) s += ",";
            s += ref(ids[i]);
        }
        s += ")";
        return s;
    }

    static std::string intList(const std::vector<int>& v)
    {
        std::string s = "(";
        for (std::size_t i = 0; i < v.size(); ++i) {
            if (i > 0) s += ",";
            s += std::to_string(v[i]);
        }
        s += ")";
        return s;
    }

    static std::string realList(const std::vector<double>& v)
    {
        std::string s = "(";
        for (std::size_t i = 0; i < v.size(); ++i) {
            if (i > 0) s += ",";
            s += fmtReal(v[i]);
        }
        s += ")";
        return s;
    }

    // ── Knot utilities ────────────────────────────────────────────────────────

    /// Compress a flat knot vector into (multiplicities, unique values).
    static std::pair<std::vector<int>, std::vector<double>>
    knotMultiplicities(const std::vector<double>& knots)
    {
        std::vector<int>    mult;
        std::vector<double> val;
        for (double k : knots) {
            if (!val.empty() && std::abs(k - val.back()) < 1e-14)
                mult.back()++;
            else {
                val.push_back(k);
                mult.push_back(1);
            }
        }
        return {mult, val};
    }

    // ── Frame builder ─────────────────────────────────────────────────────────

    static void buildFrame(const Vec3& axis, Vec3& u, Vec3& v) noexcept
    {
        Vec3 ref = (std::abs(axis.dot(Vec3::unitX())) < 0.9)
                   ? Vec3::unitX() : Vec3::unitY();
        u = (ref - axis * ref.dot(axis)).normalized();
        v = axis.cross(u);
    }

    /// Convenience overload: compute only the first frame axis.
    static void buildFrame(const Vec3& axis, Vec3& u) noexcept
    {
        Vec3 v;
        buildFrame(axis, u, v);
    }

    // ── Geometry primitives ───────────────────────────────────────────────────

    int writeCartesianPoint(const Vec3& pt)
    {
        return emit("CARTESIAN_POINT('',(" +
                    fmtReal(pt.x) + "," +
                    fmtReal(pt.y) + "," +
                    fmtReal(pt.z) + "))");
    }

    int writeDirection(const Vec3& dir)
    {
        return emit("DIRECTION('',(" +
                    fmtReal(dir.x) + "," +
                    fmtReal(dir.y) + "," +
                    fmtReal(dir.z) + "))");
    }

    int writeVector(const Vec3& dir, double mag)
    {
        Vec3 d = dir.normalized();
        int dId = writeDirection(d);
        return emit("VECTOR(''," + ref(dId) + "," + fmtReal(mag) + ")");
    }

    /// AXIS2_PLACEMENT_3D: origin + Z direction + X direction.
    int writeAxis2(const Vec3& origin, const Vec3& zDir, const Vec3& xDir)
    {
        int pId = writeCartesianPoint(origin);
        int zId = writeDirection(zDir.normalized());
        int xId = writeDirection(xDir.normalized());
        return emit("AXIS2_PLACEMENT_3D(''," +
                    ref(pId) + "," + ref(zId) + "," + ref(xId) + ")");
    }

    // ── Surface writers ───────────────────────────────────────────────────────

    int writeSurface(const ISurface& surf)
    {
        if (auto* p = dynamic_cast<const Plane*>(&surf))
            return writePlane(*p);
        if (auto* c = dynamic_cast<const Cylinder*>(&surf))
            return writeCylinder(*c);
        if (auto* c = dynamic_cast<const Cone*>(&surf))
            return writeCone(*c);
        if (auto* s = dynamic_cast<const Sphere*>(&surf))
            return writeSphere(*s);
        if (auto* t = dynamic_cast<const Torus*>(&surf))
            return writeTorus(*t);
        if (auto* d = dynamic_cast<const DiscSurface*>(&surf))
            return writeDisc(*d);
        // Check NURBS before BSpline (NURBSSurface does not derive BSplineSurface)
        if (auto* n = dynamic_cast<const NURBSSurface*>(&surf))
            return writeNURBSSurface(*n);
        if (auto* b = dynamic_cast<const BSplineSurface*>(&surf))
            return writeBSplineSurface(*b);
        // Fallback: plane at origin
        int axId = writeAxis2(Vec3::zero(), Vec3::unitZ(), Vec3::unitX());
        return emit("PLANE(''," + ref(axId) + ")");
    }

    int writePlane(const Plane& pl)
    {
        Vec3 normal = pl.uAxis().cross(pl.vAxis()).normalized();
        int axId = writeAxis2(pl.origin(), normal, pl.uAxis());
        return emit("PLANE(''," + ref(axId) + ")");
    }

    int writeCylinder(const Cylinder& cy)
    {
        int axId = writeAxis2(cy.origin(), cy.axis(), cy.uRef());
        return emit("CYLINDRICAL_SURFACE(''," + ref(axId) + "," +
                    fmtReal(cy.radius()) + ")");
    }

    int writeCone(const Cone& cn)
    {
        int axId = writeAxis2(cn.apex(), cn.axis(), cn.uRef());
        // CONICAL_SURFACE: (name, placement, radius_at_apex, semi_angle)
        return emit("CONICAL_SURFACE(''," + ref(axId) + "," +
                    fmtReal(0.0) + "," + fmtReal(cn.halfAngle()) + ")");
    }

    int writeSphere(const Sphere& sp)
    {
        int axId = writeAxis2(sp.center(), sp.axis(), sp.uRef());
        return emit("SPHERICAL_SURFACE(''," + ref(axId) + "," +
                    fmtReal(sp.radius()) + ")");
    }

    int writeTorus(const Torus& t)
    {
        int axId = writeAxis2(t.center(), t.axis(), t.uRef());
        return emit("TOROIDAL_SURFACE(''," + ref(axId) + "," +
                    fmtReal(t.majorRadius()) + "," +
                    fmtReal(t.minorRadius()) + ")");
    }

    int writeDisc(const DiscSurface& d)
    {
        // STEP has no disc-surface entity; represent as a PLANE.
        // DiscSurface natural normal is -(uRef × vRef) (see DiscSurface.h),
        // so we negate the cross product to get the correct outward direction.
        Vec3 normal = (d.uRef().cross(d.vRef()) * -1.0).normalized();
        int axId = writeAxis2(d.center(), normal, d.uRef());
        return emit("PLANE(''," + ref(axId) + ")");
    }

    int writeBSplineSurface(const BSplineSurface& bs)
    {
        int nu = bs.numU(), nv = bs.numV();

        // Control point grid as nested reference lists
        std::string ctrlStr = "(";
        for (int i = 0; i < nu; ++i) {
            if (i > 0) ctrlStr += ",";
            ctrlStr += "(";
            for (int j = 0; j < nv; ++j) {
                if (j > 0) ctrlStr += ",";
                ctrlStr += ref(writeCartesianPoint(bs.controlPoints()[i][j]));
            }
            ctrlStr += ")";
        }
        ctrlStr += ")";

        auto [multU, valU] = knotMultiplicities(bs.knotsU());
        auto [multV, valV] = knotMultiplicities(bs.knotsV());

        return emit("B_SPLINE_SURFACE_WITH_KNOTS(''," +
                    std::to_string(bs.degreeU()) + "," +
                    std::to_string(bs.degreeV()) + "," +
                    ctrlStr + ",.UNSPECIFIED.,.F.,.F.,.F.," +
                    intList(multU) + "," + intList(multV) + "," +
                    realList(valU) + "," + realList(valV) +
                    ",.UNSPECIFIED.)");
    }

    int writeNURBSSurface(const NURBSSurface& ns)
    {
        int nu = ns.numU(), nv = ns.numV();

        std::string ctrlStr = "(";
        for (int i = 0; i < nu; ++i) {
            if (i > 0) ctrlStr += ",";
            ctrlStr += "(";
            for (int j = 0; j < nv; ++j) {
                if (j > 0) ctrlStr += ",";
                ctrlStr += ref(writeCartesianPoint(ns.controlPoints()[i][j]));
            }
            ctrlStr += ")";
        }
        ctrlStr += ")";

        auto [multU, valU] = knotMultiplicities(ns.knotsU());
        auto [multV, valV] = knotMultiplicities(ns.knotsV());

        std::string wtStr = "(";
        for (int i = 0; i < nu; ++i) {
            if (i > 0) wtStr += ",";
            wtStr += "(";
            for (int j = 0; j < nv; ++j) {
                if (j > 0) wtStr += ",";
                wtStr += fmtReal(ns.weights()[i][j]);
            }
            wtStr += ")";
        }
        wtStr += ")";

        // Represent as a complex STEP entity combining B_SPLINE_SURFACE_WITH_KNOTS
        // and RATIONAL_B_SPLINE_SURFACE (AP214 complex-entity syntax).
        std::string bss =
            "B_SPLINE_SURFACE_WITH_KNOTS(''," +
            std::to_string(ns.degreeU()) + "," +
            std::to_string(ns.degreeV()) + "," +
            ctrlStr + ",.UNSPECIFIED.,.F.,.F.,.F.," +
            intList(multU) + "," + intList(multV) + "," +
            realList(valU) + "," + realList(valV) + ",.UNSPECIFIED.)";

        std::string rbs = "RATIONAL_B_SPLINE_SURFACE(" + wtStr + ")";

        return emit("(BOUNDED_SURFACE()" +
                    std::string(" ") + bss +
                    " GEOMETRIC_REPRESENTATION_ITEM()" +
                    " " + rbs +
                    " REPRESENTATION_ITEM('')" +
                    " SURFACE())");
    }

    // ── Curve writers ─────────────────────────────────────────────────────────

    int writeCurve(const ICurve3& curve)
    {
        if (auto* l = dynamic_cast<const Line3*>(&curve))
            return writeLine(*l);
        if (auto* c = dynamic_cast<const Circle3*>(&curve))
            return writeCircle(*c);
        if (auto* e = dynamic_cast<const Ellipse3*>(&curve))
            return writeEllipse(*e);
        // Check NURBS before BSpline
        if (auto* n = dynamic_cast<const NURBSCurve3*>(&curve))
            return writeNURBSCurve(*n);
        if (auto* b = dynamic_cast<const BSplineCurve3*>(&curve))
            return writeBSplineCurve(*b);
        // Fallback: infinite LINE along X at origin
        int pId = writeCartesianPoint(Vec3::zero());
        int vId = writeVector(Vec3::unitX(), 1.0);
        return emit("LINE(''," + ref(pId) + "," + ref(vId) + ")");
    }

    int writeLine(const Line3& l)
    {
        Vec3 start = l.evaluate(l.domain().lo).p;
        Vec3 dir   = l.evaluate(l.domain().lo).d1;
        double mag = dir.norm();
        if (mag < 1e-14) { dir = Vec3::unitX(); mag = 1.0; }
        int pId = writeCartesianPoint(start);
        int vId = writeVector(dir, mag);
        return emit("LINE(''," + ref(pId) + "," + ref(vId) + ")");
    }

    int writeCircle(const Circle3& c)
    {
        // Reconstruct center, xAxis, yAxis from evaluation:
        //   P(t)  = center + (xAxis*cos(t) + yAxis*sin(t))*radius
        //   d1(t) = (-xAxis*sin(t) + yAxis*cos(t))*radius
        //   d2(t) = -(xAxis*cos(t) + yAxis*sin(t))*radius
        // At t=0:  d1(0) = yAxis*radius,  d2(0) = -xAxis*radius
        auto cp0 = c.evaluate(0.0);
        double radius = cp0.d1.norm();
        if (radius < 1e-14) radius = 1.0;
        Vec3 yAxis = cp0.d1 * (1.0 / radius);  // d1(0) / radius
        Vec3 xAxis = cp0.d2 * (-1.0 / radius); // -d2(0) / radius
        Vec3 center = cp0.p - xAxis * radius;
        Vec3 normal = xAxis.cross(yAxis).normalized();
        int axId = writeAxis2(center, normal, xAxis);
        return emit("CIRCLE(''," + ref(axId) + "," + fmtReal(radius) + ")");
    }

    int writeEllipse(const Ellipse3& e)
    {
        // Reconstruct geometry from evaluation at t=0:
        //   P(t)  = center + a*cos(t)*xAxis + b*sin(t)*yAxis
        //   d1(0) = b*yAxis  →  b = |d1(0)|
        //   d2(0) = -a*xAxis →  a = |d2(0)|
        auto cp0 = e.evaluate(0.0);
        double b_val = cp0.d1.norm();
        double a_val = cp0.d2.norm();
        if (a_val < 1e-14) a_val = 1.0;
        if (b_val < 1e-14) b_val = 1.0;
        Vec3 yAxis  = cp0.d1 * (1.0 / b_val);
        Vec3 xAxis  = cp0.d2 * (-1.0 / a_val);
        Vec3 center = cp0.p - xAxis * a_val;
        Vec3 normal = xAxis.cross(yAxis).normalized();
        int axId = writeAxis2(center, normal, xAxis);
        // ELLIPSE('', axis2, semi_axis_1, semi_axis_2) — semi_axis_1 >= semi_axis_2
        if (a_val >= b_val)
            return emit("ELLIPSE(''," + ref(axId) + "," +
                        fmtReal(a_val) + "," + fmtReal(b_val) + ")");
        // Swap: use yAxis as the major axis direction so semi_axis_1 >= semi_axis_2
        Vec3 swappedNormal = yAxis.cross(xAxis).normalized();
        int swappedAxisId  = writeAxis2(center, swappedNormal, yAxis);
        return emit("ELLIPSE(''," + ref(swappedAxisId) + "," +
                    fmtReal(b_val) + "," + fmtReal(a_val) + ")");
    }

    int writeBSplineCurve(const BSplineCurve3& bc)
    {
        int n = (int)bc.controlPoints().size();
        std::string ctrlStr = "(";
        for (int i = 0; i < n; ++i) {
            if (i > 0) ctrlStr += ",";
            ctrlStr += ref(writeCartesianPoint(bc.controlPoints()[i]));
        }
        ctrlStr += ")";

        auto [mult, val] = knotMultiplicities(bc.knots());
        return emit("B_SPLINE_CURVE_WITH_KNOTS(''," +
                    std::to_string(bc.degree()) + "," +
                    ctrlStr + ",.UNSPECIFIED.,.F.,.F.," +
                    intList(mult) + "," + realList(val) +
                    ",.UNSPECIFIED.)");
    }

    int writeNURBSCurve(const NURBSCurve3& nc)
    {
        int n = (int)nc.controlPoints().size();
        std::string ctrlStr = "(";
        for (int i = 0; i < n; ++i) {
            if (i > 0) ctrlStr += ",";
            ctrlStr += ref(writeCartesianPoint(nc.controlPoints()[i]));
        }
        ctrlStr += ")";

        auto [mult, val] = knotMultiplicities(nc.knots());
        std::string wtStr = "(";
        for (int i = 0; i < n; ++i) {
            if (i > 0) wtStr += ",";
            wtStr += fmtReal(nc.weights()[i]);
        }
        wtStr += ")";

        std::string bsc =
            "B_SPLINE_CURVE_WITH_KNOTS(''," +
            std::to_string(nc.degree()) + "," +
            ctrlStr + ",.UNSPECIFIED.,.F.,.F.," +
            intList(mult) + "," + realList(val) +
            ",.UNSPECIFIED.)";
        std::string rbc = "RATIONAL_B_SPLINE_CURVE(" + wtStr + ")";

        return emit("(BOUNDED_CURVE()" +
                    std::string(" ") + bsc +
                    " CURVE()" +
                    " " + rbc +
                    " GEOMETRIC_REPRESENTATION_ITEM()" +
                    " REPRESENTATION_ITEM(''))");
    }

    // ── Synthetic edge-loop helpers ───────────────────────────────────────────
    //
    // Used when a Face carries surface geometry but no edge topology (empty
    // Wire).  Each method generates VERTEX_POINT / EDGE_CURVE / ORIENTED_EDGE
    // entities and returns the id of the resulting EDGE_LOOP.

    /// Emit a standalone VERTEX_POINT (not cached — for synthetic use only).
    int emitVertexPoint(const Vec3& pt)
    {
        int cpId = writeCartesianPoint(pt);
        return emit("VERTEX_POINT(''," + ref(cpId) + ")");
    }

    /// Emit a LINE-based EDGE_CURVE between two existing VERTEX_POINT ids.
    int emitLineEdge(int vStartId, int vEndId,
                     const Vec3& startPt, const Vec3& endPt)
    {
        Vec3   dir = endPt - startPt;
        double mag = dir.norm();
        if (mag < 1e-14) { dir = Vec3::unitX(); mag = 1.0; }
        int pId  = writeCartesianPoint(startPt);
        int vId  = writeVector(dir, mag);
        int lineId = emit("LINE(''," + ref(pId) + "," + ref(vId) + ")");
        return emit("EDGE_CURVE(''," + ref(vStartId) + "," + ref(vEndId) +
                    "," + ref(lineId) + ",.T.)");
    }

    /// Emit a CIRCLE-based EDGE_CURVE.
    /// @p axis     normal to the circle plane (used as AXIS2 Z direction).
    /// @p xRef     reference direction in the circle plane (at parameter 0).
    /// @p sameSense  .T. = counterclockwise from vStartId to vEndId.
    int emitCircleEdge(int vStartId, int vEndId,
                       const Vec3& center, const Vec3& axis, const Vec3& xRef,
                       double radius, bool sameSense = true)
    {
        int axId   = writeAxis2(center, axis, xRef);
        int circId = emit("CIRCLE(''," + ref(axId) + "," + fmtReal(radius) + ")");
        std::string ss = sameSense ? ".T." : ".F.";
        return emit("EDGE_CURVE(''," + ref(vStartId) + "," + ref(vEndId) +
                    "," + ref(circId) + "," + ss + ")");
    }

    /// Emit an ORIENTED_EDGE referencing an EDGE_CURVE id.
    int emitOrientedEdge(int ecId, bool forward)
    {
        std::string ori = forward ? ".T." : ".F.";
        return emit("ORIENTED_EDGE('',*,*," + ref(ecId) + "," + ori + ")");
    }

    /// Emit an EDGE_LOOP from a list of ORIENTED_EDGE ids.
    int emitEdgeLoop(const std::vector<int>& oeIds)
    {
        return emit("EDGE_LOOP(''," + refList(oeIds) + ")");
    }

    // ── Per-surface synthetic boundary generators ─────────────────────────────

    /// Rectangular 4-edge boundary for a Plane face with a finite UV domain.
    int synthPlaneLoop(const Plane& pl, const SurfaceDomain& dom)
    {
        double u0 = dom.u.lo, u1 = dom.u.hi;
        double v0 = dom.v.lo, v1 = dom.v.hi;
        // Reject infinite domains (e.g. unbounded planes without UV crop)
        if (std::abs(u0) > 1e9 || std::abs(u1) > 1e9 ||
            std::abs(v0) > 1e9 || std::abs(v1) > 1e9)
            return -1;

        Vec3 P00 = pl.evaluate(u0, v0).p;
        Vec3 P10 = pl.evaluate(u1, v0).p;
        Vec3 P11 = pl.evaluate(u1, v1).p;
        Vec3 P01 = pl.evaluate(u0, v1).p;

        int v00 = emitVertexPoint(P00);
        int v10 = emitVertexPoint(P10);
        int v11 = emitVertexPoint(P11);
        int v01 = emitVertexPoint(P01);

        int e0 = emitLineEdge(v00, v10, P00, P10);
        int e1 = emitLineEdge(v10, v11, P10, P11);
        int e2 = emitLineEdge(v11, v01, P11, P01);
        int e3 = emitLineEdge(v01, v00, P01, P00);

        return emitEdgeLoop({
            emitOrientedEdge(e0, true),
            emitOrientedEdge(e1, true),
            emitOrientedEdge(e2, true),
            emitOrientedEdge(e3, true)
        });
    }

    /// Seam + two-circle boundary for a full Cylinder lateral face.
    ///
    /// Topology:  seam_fwd → top_circle_fwd → seam_rev → bot_circle_rev
    /// (loop goes: V_bot → V_top → V_top → V_bot → V_bot)
    int synthCylinderLoop(const Cylinder& cy, const SurfaceDomain& dom)
    {
        double vMin = dom.v.lo, vMax = dom.v.hi;
        Vec3 axN = cy.axis().normalized();
        Vec3 uRef;
        buildFrame(axN, uRef);

        Vec3 P_bot = cy.origin() + axN * vMin + uRef * cy.radius();
        Vec3 P_top = cy.origin() + axN * vMax + uRef * cy.radius();

        int vtxBot = emitVertexPoint(P_bot);
        int vtxTop = emitVertexPoint(P_top);

        // Seam LINE from bottom seam point to top seam point
        int e_seam = emitLineEdge(vtxBot, vtxTop, P_bot, P_top);

        // Bottom circle (closed: vtxBot → vtxBot)
        Vec3 botCenter = cy.origin() + axN * vMin;
        int e_bot = emitCircleEdge(vtxBot, vtxBot, botCenter, axN, uRef, cy.radius());

        // Top circle (closed: vtxTop → vtxTop)
        Vec3 topCenter = cy.origin() + axN * vMax;
        int e_top = emitCircleEdge(vtxTop, vtxTop, topCenter, axN, uRef, cy.radius());

        return emitEdgeLoop({
            emitOrientedEdge(e_seam, true),   // V_bot → V_top
            emitOrientedEdge(e_top,  true),   // V_top → V_top  (full circle)
            emitOrientedEdge(e_seam, false),  // V_top → V_bot
            emitOrientedEdge(e_bot,  false)   // V_bot → V_bot  (full circle, rev)
        });
    }

    /// Seam + base-circle boundary for a Cone lateral face.
    ///
    /// Topology:  seam_fwd → base_circle_fwd → seam_rev
    /// (loop goes: V_apex → V_base → V_base → V_apex)
    int synthConeLoop(const Cone& cn, const SurfaceDomain& dom)
    {
        double vMax = dom.v.hi;
        Vec3 axN = cn.axis().normalized();
        Vec3 uRef;
        buildFrame(axN, uRef);

        double baseR = vMax * std::tan(cn.halfAngle());
        Vec3 P_apex = cn.apex();
        Vec3 P_base = cn.apex() + axN * vMax + uRef * baseR;

        int vtxApex = emitVertexPoint(P_apex);
        int vtxBase = emitVertexPoint(P_base);

        // Seam LINE: apex → base seam point
        int e_seam = emitLineEdge(vtxApex, vtxBase, P_apex, P_base);

        // Base circle (closed: vtxBase → vtxBase)
        Vec3 baseCenter = cn.apex() + axN * vMax;
        int e_base = emitCircleEdge(vtxBase, vtxBase, baseCenter, axN, uRef, baseR);

        return emitEdgeLoop({
            emitOrientedEdge(e_seam, true),   // apex → base seam pt
            emitOrientedEdge(e_base, true),   // base seam pt → base seam pt
            emitOrientedEdge(e_seam, false)   // base seam pt → apex
        });
    }

    /// Outer circle boundary for a full DiscSurface face (rInner ≈ 0).
    int synthDiscLoop(const DiscSurface& d, const SurfaceDomain& dom)
    {
        double rOuter = dom.v.hi;
        // Seam point on outer rim at u=0
        Vec3 seamPt = d.center() + d.uRef() * rOuter;
        int vtx = emitVertexPoint(seamPt);

        // DiscSurface natural normal is -(uRef × vRef); use that as circle axis
        Vec3 discAxis = (d.uRef().cross(d.vRef()) * -1.0).normalized();
        // Outer circle (closed: vtx → vtx)
        int e_outer = emitCircleEdge(vtx, vtx, d.center(), discAxis, d.uRef(), rOuter);

        return emitEdgeLoop({ emitOrientedEdge(e_outer, true) });
    }

    /// Seam-meridian boundary for a Sphere face.
    ///
    /// Uses a single great-circle arc (the seam at u=0) traversed both ways
    /// so that the loop encircles the whole sphere surface.
    ///
    /// CIRCLE axis = +Y, ref = +X (assuming sphere axis = +Z, uRef = +X):
    ///   P(t) = center + r*(cos(t)*X + sin(t)*(Y×X))
    ///        = center + r*(cos(t)*X - sin(t)*Z)
    ///   P(π/2)  = center - r*Z  (south pole)
    ///   P(-π/2) = center + r*Z  (north pole)
    ///   P(0)    = center + r*X  (equator seam point)
    ///
    /// With same_sense=.F. on the EDGE_CURVE, traversal goes clockwise
    /// (decreasing t), so the arc from south (t=π/2) to north (t=-π/2)
    /// passes through the east equator at t=0 — matching the seam at u=0.
    int synthSphereLoop(const Sphere& sp, const SurfaceDomain& /*dom*/)
    {
        Vec3 southPole = sp.center() + Vec3{0.0, 0.0, -sp.radius()};
        Vec3 northPole = sp.center() + Vec3{0.0, 0.0,  sp.radius()};

        int vtxSouth = emitVertexPoint(southPole);
        int vtxNorth = emitVertexPoint(northPole);

        // Seam circle in the XZ plane (axis = +Y, ref = +X), same_sense = .F.
        // so the short arc (east side) is taken from south pole to north pole.
        int e_seam = emitCircleEdge(vtxSouth, vtxNorth,
                                    sp.center(),
                                    Vec3::unitY(),   // circle axis
                                    Vec3::unitX(),   // reference direction (t=0)
                                    sp.radius(),
                                    false);          // same_sense = .F. → east-side arc

        // Loop: seam forward (south→north via east) + seam reversed (north→south via west)
        return emitEdgeLoop({
            emitOrientedEdge(e_seam, true),
            emitOrientedEdge(e_seam, false)
        });
    }

    /// Double-seam boundary for a Torus face.
    ///
    /// Two seam circles share a single VERTEX_POINT at P(0,0):
    ///   major circle  – u varies at v=0, radius = R+r
    ///   minor circle  – v varies at u=0, radius = r
    ///
    /// Loop: major_fwd + minor_fwd + major_rev + minor_rev
    int synthTorusLoop(const Torus& t, const SurfaceDomain& /*dom*/)
    {
        Vec3 uRef;
        buildFrame(t.axis(), uRef);

        // Single shared vertex at P(0, 0)
        Vec3 V1pt = t.center() + uRef * (t.majorRadius() + t.minorRadius());
        int vtx1 = emitVertexPoint(V1pt);

        // Major circle: u varies at v=0
        //   P(u,0) = center + (R+r)*(cos(u)*uRef + sin(u)*vRef)
        //   CIRCLE axis = t.axis(), ref = uRef, radius = R+r
        int e_major = emitCircleEdge(vtx1, vtx1,
                                     t.center(), t.axis(), uRef,
                                     t.majorRadius() + t.minorRadius());

        // Minor circle: v varies at u=0
        //   P(0,v) = center + (R+r*cos(v))*uRef + r*sin(v)*axis
        //   = tubeCenter + r*(cos(v)*uRef + sin(v)*axis)
        //   CIRCLE axis = uRef × axis (perpendicular to the meridional plane),
        //                 ref = uRef, radius = r
        //
        // Derivation:  STEP circle P(t) = origin + r*(cos(t)*D + sin(t)*(A×D))
        //   Set A×D = axis  →  A = uRef × axis (since (uRef×axis)×uRef = axis
        //   when uRef⊥axis: verify with axis=Z, uRef=X → (X×Z)×X = (-Y)×X = Z ✓)
        Vec3 tubeCenter  = t.center() + uRef * t.majorRadius();
        Vec3 minorAxis   = uRef.cross(t.axis()).normalized(); // ≡ −vRef
        int e_minor = emitCircleEdge(vtx1, vtx1,
                                     tubeCenter, minorAxis, uRef,
                                     t.minorRadius());

        return emitEdgeLoop({
            emitOrientedEdge(e_major, true),
            emitOrientedEdge(e_minor, true),
            emitOrientedEdge(e_major, false),
            emitOrientedEdge(e_minor, false)
        });
    }

    /// Dispatch to the appropriate synthetic boundary generator.
    /// Returns the EDGE_LOOP id, or -1 if no synthetic loop can be generated.
    int synthEdgeLoop(const Face& face)
    {
        if (!face.hasSurface()) return -1;

        SurfaceDomain dom = face.uvDomain();
        const ISurface* surf = face.surface().get();

        if (auto* pl = dynamic_cast<const Plane*>(surf))
            return synthPlaneLoop(*pl, dom);
        if (auto* cy = dynamic_cast<const Cylinder*>(surf))
            return synthCylinderLoop(*cy, dom);
        if (auto* co = dynamic_cast<const Cone*>(surf))
            return synthConeLoop(*co, dom);
        if (auto* di = dynamic_cast<const DiscSurface*>(surf))
            return synthDiscLoop(*di, dom);
        if (auto* sp = dynamic_cast<const Sphere*>(surf))
            return synthSphereLoop(*sp, dom);
        if (auto* to = dynamic_cast<const Torus*>(surf))
            return synthTorusLoop(*to, dom);
        // BSplineSurface / NURBSSurface: no generic boundary synthesis
        return -1;
    }

    // ── Topology writers ──────────────────────────────────────────────────────

    std::unordered_map<uint64_t, int> vertexStepId_; // Vertex id → VERTEX_POINT
    std::unordered_map<uint64_t, int> edgeStepId_;   // Edge id   → EDGE_CURVE

    int writeVertexPoint(const Vertex& v)
    {
        uint64_t key = v.id().value();
        auto it = vertexStepId_.find(key);
        if (it != vertexStepId_.end()) return it->second;
        int cpId = writeCartesianPoint(v.point());
        int id = emit("VERTEX_POINT(''," + ref(cpId) + ")");
        vertexStepId_[key] = id;
        return id;
    }

    int writeEdgeCurve(const Edge& e)
    {
        uint64_t key = e.id().value();
        auto it = edgeStepId_.find(key);
        if (it != edgeStepId_.end()) return it->second;

        // Vertex points
        int vStartId, vEndId;
        if (e.start()) {
            vStartId = writeVertexPoint(*e.start());
        } else {
            int cpId = writeCartesianPoint(Vec3::zero());
            vStartId = emit("VERTEX_POINT(''," + ref(cpId) + ")");
        }
        if (e.end()) {
            vEndId = writeVertexPoint(*e.end());
        } else {
            int cpId = writeCartesianPoint(Vec3::zero());
            vEndId = emit("VERTEX_POINT(''," + ref(cpId) + ")");
        }

        // Curve geometry
        int curveId;
        if (e.hasCurve()) {
            curveId = writeCurve(*e.curve());
        } else if (e.start() && e.end()) {
            // Fallback: straight LINE between start and end vertices
            Vec3 d = e.end()->point() - e.start()->point();
            double mag = d.norm();
            int pId = writeCartesianPoint(e.start()->point());
            int vId = writeVector(mag > 1e-14 ? d : Vec3::unitX(),
                                  mag > 1e-14 ? mag : 1.0);
            curveId = emit("LINE(''," + ref(pId) + "," + ref(vId) + ")");
        } else {
            int pId = writeCartesianPoint(Vec3::zero());
            int vId = writeVector(Vec3::unitX(), 1.0);
            curveId = emit("LINE(''," + ref(pId) + "," + ref(vId) + ")");
        }

        int id = emit("EDGE_CURVE(''," + ref(vStartId) + "," + ref(vEndId) +
                      "," + ref(curveId) + ",.T.)");
        edgeStepId_[key] = id;
        return id;
    }

    int writeOrientedEdge(const CoEdge& ce)
    {
        int ecId = writeEdgeCurve(*ce.edge());
        std::string orient =
            (ce.orientation() == CoEdgeOrientation::kForward) ? ".T." : ".F.";
        return emit("ORIENTED_EDGE('',*,*," + ref(ecId) + "," + orient + ")");
    }

    int writeEdgeLoop(const Wire& wire)
    {
        std::vector<int> oeIds;
        for (auto& ce : wire.coEdges()) {
            if (!ce || !ce->edge()) continue;
            oeIds.push_back(writeOrientedEdge(*ce));
        }
        return emit("EDGE_LOOP(''," + refList(oeIds) + ")");
    }

    int writeAdvancedFace(const Face& face)
    {
        // Surface geometry
        int surfId;
        if (face.hasSurface()) {
            surfId = writeSurface(*face.surface());
        } else {
            int axId = writeAxis2(Vec3::zero(), Vec3::unitZ(), Vec3::unitX());
            surfId = emit("PLANE(''," + ref(axId) + ")");
        }

        // Face bounds (outer + inner wires)
        std::vector<int> boundIds;

        bool outerHasEdges = face.outerWire() &&
                             !face.outerWire()->coEdges().empty();

        std::string outerOriStr = face.outerWireOrientation() ? ".T." : ".F.";

        if (outerHasEdges) {
            // Use the existing topological wire
            int loopId = writeEdgeLoop(*face.outerWire());
            boundIds.push_back(
                emit("FACE_OUTER_BOUND(''," + ref(loopId) + "," + outerOriStr + ")"));
        } else {
            // No edge topology: synthesize a boundary from the surface geometry
            int loopId = synthEdgeLoop(face);
            if (loopId > 0) {
                boundIds.push_back(
                    emit("FACE_OUTER_BOUND(''," + ref(loopId) + ",.T.)"));
            }
        }

        const auto& innerOris = face.innerWireOrientations();
        const auto& innerWires = face.innerWires();
        for (std::size_t i = 0; i < innerWires.size(); ++i) {
            const auto& iw = innerWires[i];
            if (!iw || iw->coEdges().empty()) continue;
            int loopId = writeEdgeLoop(*iw);
            bool ori = (i < innerOris.size()) ? innerOris[i] : false;
            std::string oriStr = ori ? ".T." : ".F.";
            boundIds.push_back(
                emit("FACE_BOUND(''," + ref(loopId) + "," + oriStr + ")"));
        }

        std::string senseStr =
            (face.orientation() == FaceOrientation::kForward) ? ".T." : ".F.";
        return emit("ADVANCED_FACE(''," + refList(boundIds) +
                    "," + ref(surfId) + "," + senseStr + ")");
    }

    // ── Body traversal ────────────────────────────────────────────────────────

    std::vector<int> brepIds_;

    void buildEntities(const Body& body)
    {
        for (auto& lump : body.lumps()) {
            if (!lump) continue;
            buildLump(*lump);
        }
    }

    void buildLump(const Lump& lump)
    {
        if (!lump.outerShell()) return;
        int shellId = buildShell(*lump.outerShell());
        int id = emit("MANIFOLD_SOLID_BREP(''," + ref(shellId) + ")");
        brepIds_.push_back(id);
    }

    int buildShell(const Shell& shell)
    {
        std::vector<int> faceIds;
        for (auto& face : shell.faces()) {
            if (!face) continue;
            faceIds.push_back(writeAdvancedFace(*face));
        }
        if (shell.isClosed())
            return emit("CLOSED_SHELL(''," + refList(faceIds) + ")");
        else
            return emit("OPEN_SHELL(''," + refList(faceIds) + ")");
    }

    // ── STEP file assembly ────────────────────────────────────────────────────

    std::string generate(const std::string& name)
    {
        std::ostringstream os;

        // ── Header ────────────────────────────────────────────────────────────
        os << "ISO-10303-21;\n";
        os << "HEADER;\n";
        os << "FILE_DESCRIPTION(('GectorKernel STEP Export'),'2;1');\n";
        os << "FILE_NAME('" << name
           << "','2025-01-01T00:00:00',(''),(''),'GectorKernel','','');\n";
        os << "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 1 }'));\n";
        os << "ENDSEC;\n";
        os << "DATA;\n";

        // ── Geometry / topology entities ──────────────────────────────────────
        for (auto& ent : entities_)
            os << "#" << ent.id << "=" << ent.text << ";\n";

        // ── Product and geometric-context entities ────────────────────────────
        // IDs continue from where geometry left off.
        int appCtxId  = nextId_++;
        int prodCtxId = nextId_++;
        int prodId    = nextId_++;
        int pdfId     = nextId_++;
        int pdctxId   = nextId_++;
        int pddId     = nextId_++;
        int pdsId     = nextId_++;
        int lenUnitId = nextId_++;
        int angUnitId = nextId_++;
        int saUnitId  = nextId_++;
        int uncMeasId = nextId_++;
        int geomCtxId = nextId_++;
        int absrId    = nextId_++;
        int sdrId     = nextId_++;

        os << "#" << appCtxId
           << "=APPLICATION_CONTEXT('mechanical design');\n";
        os << "#" << prodCtxId
           << "=PRODUCT_CONTEXT('',#" << appCtxId << ",'mechanical');\n";
        os << "#" << prodId
           << "=PRODUCT('" << name << "','" << name << "','',(#"
           << prodCtxId << "));\n";
        os << "#" << pdfId
           << "=PRODUCT_DEFINITION_FORMATION('','',#" << prodId << ");\n";
        os << "#" << pdctxId
           << "=PRODUCT_DEFINITION_CONTEXT('part definition',#"
           << appCtxId << ",'design');\n";
        os << "#" << pddId
           << "=PRODUCT_DEFINITION('design','',#" << pdfId
           << ",#" << pdctxId << ");\n";
        os << "#" << pdsId
           << "=PRODUCT_DEFINITION_SHAPE('','',#" << pddId << ");\n";
        os << "#" << lenUnitId
           << "=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n";
        os << "#" << angUnitId
           << "=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n";
        os << "#" << saUnitId
           << "=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());\n";
        os << "#" << uncMeasId
           << "=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-007),#"
           << lenUnitId << ",'distance_accuracy_value','');\n";
        os << "#" << geomCtxId
           << "=(GEOMETRIC_REPRESENTATION_CONTEXT(3)"
              "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" << uncMeasId << "))"
              "GLOBAL_UNIT_ASSIGNED_CONTEXT((#" << lenUnitId
           << ",#" << angUnitId << ",#" << saUnitId << "))"
              "REPRESENTATION_CONTEXT('',''));\n";

        // Build the BREP id list
        std::string brepList = "(";
        for (std::size_t i = 0; i < brepIds_.size(); ++i) {
            if (i > 0) brepList += ",";
            brepList += "#" + std::to_string(brepIds_[i]);
        }
        brepList += ")";

        os << "#" << absrId
           << "=ADVANCED_BREP_SHAPE_REPRESENTATION(''," << brepList
           << ",#" << geomCtxId << ");\n";
        os << "#" << sdrId
           << "=SHAPE_DEFINITION_REPRESENTATION(#" << pdsId
           << ",#" << absrId << ");\n";

        os << "ENDSEC;\n";
        os << "END-ISO-10303-21;\n";

        return os.str();
    }
};

} // namespace gk
