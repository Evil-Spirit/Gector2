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
        Vec3 xRef, dummy;
        buildFrame(cy.axis(), xRef, dummy);
        int axId = writeAxis2(cy.origin(), cy.axis(), xRef);
        return emit("CYLINDRICAL_SURFACE(''," + ref(axId) + "," +
                    fmtReal(cy.radius()) + ")");
    }

    int writeCone(const Cone& cn)
    {
        Vec3 xRef, dummy;
        buildFrame(cn.axis(), xRef, dummy);
        int axId = writeAxis2(cn.apex(), cn.axis(), xRef);
        // CONICAL_SURFACE: (name, placement, radius_at_apex, semi_angle)
        return emit("CONICAL_SURFACE(''," + ref(axId) + "," +
                    fmtReal(0.0) + "," + fmtReal(cn.halfAngle()) + ")");
    }

    int writeSphere(const Sphere& sp)
    {
        Vec3 xRef, dummy;
        buildFrame(Vec3::unitZ(), xRef, dummy);
        int axId = writeAxis2(sp.center(), Vec3::unitZ(), xRef);
        return emit("SPHERICAL_SURFACE(''," + ref(axId) + "," +
                    fmtReal(sp.radius()) + ")");
    }

    int writeTorus(const Torus& t)
    {
        Vec3 xRef, dummy;
        buildFrame(t.axis(), xRef, dummy);
        int axId = writeAxis2(t.center(), t.axis(), xRef);
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
        if (face.outerWire()) {
            int loopId = writeEdgeLoop(*face.outerWire());
            boundIds.push_back(
                emit("FACE_OUTER_BOUND(''," + ref(loopId) + ",.T.)"));
        }
        for (auto& iw : face.innerWires()) {
            if (!iw) continue;
            int loopId = writeEdgeLoop(*iw);
            boundIds.push_back(
                emit("FACE_BOUND(''," + ref(loopId) + ",.T.)"));
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
