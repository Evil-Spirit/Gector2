#pragma once
// Chapter 9.2 — STEP AP203/AP214 Import.
//
// Parses an ISO 10303-21 STEP text string and builds gk::Body objects.
//
// Supported geometry:
//   Surfaces: Plane, Sphere, Cylinder, Cone, Torus,
//             BSplineSurface (standalone and compound), NURBSSurface (compound).
//   Curves:   Line3, Circle3, BSplineCurve3 (standalone), NURBSCurve3 (compound),
//             QuasiUniformCurve (treated as BSpline), SurfaceCurve/IntersectionCurve
//             (3-D curve extracted).
//   Topology: Vertex, Edge (with curve binding), CoEdge, Wire, Face (with surface),
//             Shell (closed/open), Lump, Body.
//
// Usage:
//   std::string text = ...; // ISO 10303-21 text
//   auto bodies = gk::StepReader::read(text);

#include "gk/brep/Body.h"
#include "gk/brep/CoEdge.h"
#include "gk/curve/BSplineCurve.h"
#include "gk/curve/Circle.h"
#include "gk/curve/Line.h"
#include "gk/curve/NURBSCurve.h"
#include "gk/surface/BSplineSurface.h"
#include "gk/surface/Cone.h"
#include "gk/surface/Cylinder.h"
#include "gk/surface/NURBSSurface.h"
#include "gk/surface/Plane.h"
#include "gk/surface/Sphere.h"
#include "gk/surface/Torus.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace gk {

class StepReader
{
public:
    /// Parse STEP text and return all bodies found.
    static std::vector<Handle<Body>> read(const std::string& text)
    {
        StepReader reader;
        reader.parse(text);
        return reader.buildBodies();
    }

    /// Parse STEP text and return first body (nullptr if none found).
    static Handle<Body> readOne(const std::string& text)
    {
        auto bodies = read(text);
        return bodies.empty() ? Handle<Body>{} : bodies.front();
    }

private:
    // ── Raw entity record ─────────────────────────────────────────────────────

    struct Part { std::string type; std::string body; };

    struct Entity {
        std::string       type;    // empty for complex entities
        std::string       body;    // content inside outermost parens
        bool              complex{false};
        std::vector<Part> parts;   // for complex entities
    };

    std::unordered_map<int, Entity> entities_;
    double                          angConv_{1.0}; // 1.0=radians, pi/180=degrees

    // ── Resolved geometry caches ──────────────────────────────────────────────

    std::unordered_map<int, Vec3>                      vec3Cache_;
    std::unordered_map<int, std::shared_ptr<ISurface>> surfCache_;
    std::unordered_map<int, std::shared_ptr<ICurve3>>  curv3Cache_;
    std::unordered_map<int, Handle<Vertex>>            vxCache_;
    std::unordered_map<int, Handle<Edge>>              edgeCache_;
    std::unordered_map<int, Handle<Wire>>              wireCache_;
    std::unordered_map<int, Handle<Face>>              faceCache_;
    std::unordered_map<int, Handle<Shell>>             shellCache_;

    // ── Parsing ───────────────────────────────────────────────────────────────

    void parse(const std::string& text)
    {
        // 1. Strip block comments /* ... */
        std::string clean;
        clean.reserve(text.size());
        std::size_t i = 0;
        while (i < text.size()) {
            if (i + 1 < text.size() && text[i] == '/' && text[i + 1] == '*') {
                i += 2;
                while (i + 1 < text.size() &&
                       !(text[i] == '*' && text[i + 1] == '/'))
                    ++i;
                i += 2;
            } else {
                clean += text[i++];
            }
        }

        // 2. Split at ';', flatten whitespace inside each statement
        std::string stmt;
        stmt.reserve(256);
        for (char c : clean) {
            if (c == ';') {
                processStatement(stmt);
                stmt.clear();
            } else {
                stmt += (c == '\n' || c == '\r' || c == '\t') ? ' ' : c;
            }
        }

        // 3. Detect angular units
        detectAngleUnit();
    }

    void detectAngleUnit()
    {
        // Look for a CONVERSION_BASED_UNIT whose name contains "DEGREE".
        // This entity can appear standalone or as a part of a complex entity.
        for (auto& kv : entities_) {
            auto checkName = [&](const std::string& type, const std::string& body) {
                if (type != "CONVERSION_BASED_UNIT") return;
                // body: 'DEGREES', #ref  — check the string argument
                std::string up = body;
                std::transform(up.begin(), up.end(), up.begin(),
                               [](unsigned char c) { return (char)std::toupper(c); });
                if (up.find("DEGREE") != std::string::npos) {
                    constexpr double kPi = 3.14159265358979323846;
                    angConv_ = kPi / 180.0;
                }
            };
            if (kv.second.complex) {
                for (auto& p : kv.second.parts)
                    checkName(p.type, p.body);
            } else {
                checkName(kv.second.type, kv.second.body);
            }
        }
    }

    // ── Statement processing ──────────────────────────────────────────────────

    void processStatement(const std::string& raw)
    {
        std::string s = trim(raw);
        if (s.empty() || s.front() != '#') return;

        std::size_t eq = s.find('=');
        if (eq == std::string::npos) return;

        int id = 0;
        try { id = std::stoi(s.substr(1, eq - 1)); } catch (...) { return; }

        std::string rest = trim(s.substr(eq + 1));
        if (rest.empty()) return;

        Entity rec;
        if (rest.front() == '(') {
            rec.complex = true;
            parseComplexEntity(rest, rec);
        } else {
            std::size_t lp = rest.find('(');
            if (lp == std::string::npos) {
                rec.type = trim(rest);
            } else {
                rec.type = trim(rest.substr(0, lp));
                rec.body = extractParens(rest, lp);
            }
        }
        entities_[id] = std::move(rec);
    }

    void parseComplexEntity(const std::string& s, Entity& rec)
    {
        std::string content = extractParens(s, 0);
        std::size_t i = 0;
        while (i < content.size()) {
            while (i < content.size() && content[i] == ' ') ++i;
            if (i >= content.size()) break;

            // Read type name: letters, digits, underscores
            std::size_t ts = i;
            while (i < content.size() && content[i] != '(' &&
                   content[i] != ')' && content[i] != ' ')
                ++i;
            std::string typeName = trim(content.substr(ts, i - ts));
            if (typeName.empty()) { ++i; continue; }

            while (i < content.size() && content[i] == ' ') ++i;

            std::string partBody;
            if (i < content.size() && content[i] == '(') {
                partBody = extractParens(content, i);
                // Advance i past the matching ')'
                int depth = 0;
                while (i < content.size()) {
                    char c = content[i++];
                    if (c == '\'') {
                        while (i < content.size() && content[i++] != '\'') {}
                    } else if (c == '(') {
                        ++depth;
                    } else if (c == ')') {
                        if (--depth == 0) break;
                    }
                }
            }
            rec.parts.push_back({std::move(typeName), std::move(partBody)});
        }
    }

    // ── String utilities ──────────────────────────────────────────────────────

    static std::string trim(const std::string& s)
    {
        std::size_t b = s.find_first_not_of(" \t\r\n");
        if (b == std::string::npos) return {};
        std::size_t e = s.find_last_not_of(" \t\r\n");
        return s.substr(b, e - b + 1);
    }

    /// Return the content inside the matching '(' starting at `start` in `s`.
    static std::string extractParens(const std::string& s, std::size_t start)
    {
        if (start >= s.size() || s[start] != '(') return {};
        int depth = 0;
        std::size_t b = start + 1;
        std::size_t i = start;
        while (i < s.size()) {
            char c = s[i];
            if (c == '\'') {
                ++i;
                while (i < s.size() && s[i] != '\'') ++i;
                ++i;
            } else if (c == '(') {
                ++depth; ++i;
            } else if (c == ')') {
                if (--depth == 0) return s.substr(b, i - b);
                ++i;
            } else {
                ++i;
            }
        }
        return s.substr(b); // malformed; return remainder
    }

    // ── Argument splitting ────────────────────────────────────────────────────

    /// Split `body` at top-level commas (depth 0, outside string literals).
    static std::vector<std::string> splitArgs(const std::string& body)
    {
        std::vector<std::string> result;
        std::string cur;
        int  depth = 0;
        bool inStr = false;
        for (char c : body) {
            if (inStr) {
                cur += c;
                if (c == '\'') inStr = false;
            } else if (c == '\'') {
                cur += c; inStr = true;
            } else if (c == '(') {
                ++depth; cur += c;
            } else if (c == ')') {
                --depth; cur += c;
            } else if (c == ',' && depth == 0) {
                result.push_back(trim(cur)); cur.clear();
            } else {
                cur += c;
            }
        }
        auto last = trim(cur);
        if (!last.empty()) result.push_back(last);
        return result;
    }

    // ── Low-level token parsers ───────────────────────────────────────────────

    static int parseRef(const std::string& tok)
    {
        std::string t = trim(tok);
        if (!t.empty() && t.front() == '#') {
            try { return std::stoi(t.substr(1)); } catch (...) {}
        }
        return -1;
    }

    static double parseReal(const std::string& tok)
    {
        try { return std::stod(trim(tok)); } catch (...) { return 0.0; }
    }

    static int parseInt(const std::string& tok)
    {
        try { return std::stoi(trim(tok)); } catch (...) { return 0; }
    }

    static std::vector<int> parseRefList(const std::string& tok)
    {
        std::string t = trim(tok);
        if (t.empty() || t.front() != '(') return {};
        auto args = splitArgs(extractParens(t, 0));
        std::vector<int> result;
        for (auto& a : args) {
            int r = parseRef(a);
            if (r >= 0) result.push_back(r);
        }
        return result;
    }

    static std::vector<double> parseRealList(const std::string& tok)
    {
        std::string t = trim(tok);
        if (t.empty() || t.front() != '(') return {};
        auto args = splitArgs(extractParens(t, 0));
        std::vector<double> result;
        for (auto& a : args) result.push_back(parseReal(a));
        return result;
    }

    static std::vector<int> parseIntList(const std::string& tok)
    {
        std::string t = trim(tok);
        if (t.empty() || t.front() != '(') return {};
        auto args = splitArgs(extractParens(t, 0));
        std::vector<int> result;
        for (auto& a : args) result.push_back(parseInt(a));
        return result;
    }

    /// Parse "((#1,#2),(#3,#4))" → vector<vector<int>>
    static std::vector<std::vector<int>> parseRefListList(const std::string& tok)
    {
        std::string t = trim(tok);
        if (t.empty() || t.front() != '(') return {};
        std::string outer = extractParens(t, 0);
        std::vector<std::vector<int>> result;
        std::string cur;
        int depth = 0;
        for (char c : outer) {
            if (c == '(') {
                ++depth; cur += c;
            } else if (c == ')') {
                --depth; cur += c;
                if (depth == 0) {
                    result.push_back(parseRefList(trim(cur)));
                    cur.clear();
                }
            } else if (c == ',' && depth == 0) {
                // top-level separator between inner lists; ignore
            } else {
                cur += c;
            }
        }
        return result;
    }

    /// Parse "((1.,2.),(3.,4.))" → vector<vector<double>>
    static std::vector<std::vector<double>> parseRealListList(const std::string& tok)
    {
        std::string t = trim(tok);
        if (t.empty() || t.front() != '(') return {};
        std::string outer = extractParens(t, 0);
        std::vector<std::vector<double>> result;
        std::string cur;
        int depth = 0;
        for (char c : outer) {
            if (c == '(') {
                ++depth; cur += c;
            } else if (c == ')') {
                --depth; cur += c;
                if (depth == 0) {
                    result.push_back(parseRealList(trim(cur)));
                    cur.clear();
                }
            } else if (c == ',' && depth == 0) {
                // separator between inner lists; ignore
            } else {
                cur += c;
            }
        }
        return result;
    }

    // ── Geometry helpers ──────────────────────────────────────────────────────

    const Entity* getEntity(int id) const
    {
        auto it = entities_.find(id);
        return (it != entities_.end()) ? &it->second : nullptr;
    }

    Vec3 resolveVec3(int id)
    {
        auto it = vec3Cache_.find(id);
        if (it != vec3Cache_.end()) return it->second;

        const Entity* rec = getEntity(id);
        if (!rec) return Vec3::zero();

        // Works for both CARTESIAN_POINT and DIRECTION
        auto args = splitArgs(rec->body);
        if (args.size() >= 2) {
            auto coords = parseRealList(args[1]);
            if (coords.size() >= 3) {
                Vec3 v{coords[0], coords[1], coords[2]};
                vec3Cache_[id] = v;
                return v;
            }
        }
        return Vec3::zero();
    }

    struct Axis2 { Vec3 origin, zDir, xDir; };

    Axis2 resolveAxis2(int id)
    {
        const Entity* rec = getEntity(id);
        if (!rec || rec->type != "AXIS2_PLACEMENT_3D")
            return {Vec3::zero(), Vec3::unitZ(), Vec3::unitX()};

        auto args = splitArgs(rec->body);
        // args: name, origin_ref, z_dir_ref, [x_dir_ref or $]
        Vec3 origin = (args.size() >= 2) ? resolveVec3(parseRef(args[1])) : Vec3::zero();
        Vec3 zDir   = Vec3::unitZ();
        Vec3 xDir   = Vec3::unitX();

        if (args.size() >= 3) {
            int zRef = parseRef(args[2]);
            if (zRef >= 0) {
                Vec3 z = resolveVec3(zRef);
                if (z.squaredNorm() > 1e-20) zDir = z.normalized();
            }
        }
        if (args.size() >= 4) {
            int xRef = parseRef(args[3]);
            if (xRef >= 0) {
                Vec3 x = resolveVec3(xRef);
                if (x.squaredNorm() > 1e-20) {
                    // Orthogonalise w.r.t. zDir
                    x = x - zDir * zDir.dot(x);
                    if (x.squaredNorm() > 1e-20) xDir = x.normalized();
                }
            }
        }
        if (xDir.squaredNorm() < 1e-20 ||
            std::abs(xDir.dot(zDir)) > 0.99) {
            // Build a perpendicular frame
            Vec3 ref = std::abs(zDir.dot(Vec3::unitX())) < 0.9
                       ? Vec3::unitX() : Vec3::unitY();
            xDir = (ref - zDir * ref.dot(zDir)).normalized();
        }
        return {origin, zDir, xDir};
    }

    static std::vector<double> expandKnots(const std::vector<int>& mults,
                                            const std::vector<double>& vals)
    {
        std::vector<double> knots;
        for (std::size_t k = 0; k < mults.size() && k < vals.size(); ++k)
            for (int m = 0; m < mults[k]; ++m)
                knots.push_back(vals[k]);
        return knots;
    }

    // ── Curve resolvers ───────────────────────────────────────────────────────

    std::shared_ptr<ICurve3> resolveCurve(int id)
    {
        auto it = curv3Cache_.find(id);
        if (it != curv3Cache_.end()) return it->second;

        const Entity* rec = getEntity(id);
        if (!rec) return nullptr;

        std::shared_ptr<ICurve3> result;

        if (rec->complex) {
            result = resolveCompoundCurve(*rec);
        } else if (rec->type == "LINE") {
            result = resolveLine(*rec);
        } else if (rec->type == "CIRCLE") {
            result = resolveCircle(*rec);
        } else if (rec->type == "B_SPLINE_CURVE_WITH_KNOTS") {
            result = resolveBSplineCurve(*rec);
        } else if (rec->type == "QUASI_UNIFORM_CURVE") {
            result = resolveQuasiUniformCurve(*rec);
        } else if (rec->type == "SURFACE_CURVE" ||
                   rec->type == "INTERSECTION_CURVE" ||
                   rec->type == "SEAM_CURVE") {
            // Use the 3-D curve (second argument after name)
            auto args = splitArgs(rec->body);
            if (args.size() >= 2) {
                int innerRef = parseRef(args[1]);
                if (innerRef >= 0) result = resolveCurve(innerRef);
            }
        }

        curv3Cache_[id] = result;
        return result;
    }

    std::shared_ptr<ICurve3> resolveCompoundCurve(const Entity& rec)
    {
        int               degree = 1;
        std::vector<Vec3> ctrlPts;
        std::vector<int>  mults;
        std::vector<double> knots, weights;
        bool hasKnots = false, hasRational = false, isQuasiUniform = false;

        for (const auto& part : rec.parts) {
            if (part.type == "B_SPLINE_CURVE") {
                auto a = splitArgs(part.body);
                if (a.size() >= 1) degree = parseInt(a[0]);
                if (a.size() >= 2) {
                    for (int r : parseRefList(a[1]))
                        ctrlPts.push_back(resolveVec3(r));
                }
            } else if (part.type == "B_SPLINE_CURVE_WITH_KNOTS") {
                auto a = splitArgs(part.body);
                if (a.size() >= 2) {
                    mults  = parseIntList(a[0]);
                    knots  = parseRealList(a[1]);
                    hasKnots = true;
                }
            } else if (part.type == "RATIONAL_B_SPLINE_CURVE") {
                auto a = splitArgs(part.body);
                if (!a.empty()) { weights = parseRealList(a[0]); hasRational = true; }
            } else if (part.type == "QUASI_UNIFORM_CURVE") {
                // Mixin with no explicit knots → generate quasi-uniform knot vector
                isQuasiUniform = true;
            }
        }

        if (ctrlPts.empty()) return nullptr;

        // Build knot vector: explicit knots take priority; fall back to quasi-uniform
        std::vector<double> flat;
        if (hasKnots) {
            flat = expandKnots(mults, knots);
        } else if (isQuasiUniform) {
            flat = BSplineCurve3::uniformKnots((int)ctrlPts.size(), degree);
        } else {
            return nullptr;
        }

        if (hasRational && !weights.empty()) {
            try { return std::make_shared<NURBSCurve3>(degree, flat, ctrlPts, weights); }
            catch (...) {}
        }
        try { return std::make_shared<BSplineCurve3>(degree, flat, ctrlPts); }
        catch (...) {}
        return nullptr;
    }

    std::shared_ptr<ICurve3> resolveLine(const Entity& rec)
    {
        // LINE('', origin_pt, vector)
        auto a = splitArgs(rec.body);
        if (a.size() < 3) return nullptr;
        Vec3 origin = resolveVec3(parseRef(a[1]));
        Vec3 dir    = Vec3::unitX();
        int vecRef = parseRef(a[2]);
        if (vecRef >= 0) {
            const Entity* vr = getEntity(vecRef);
            if (vr && vr->type == "VECTOR") {
                auto va = splitArgs(vr->body);
                if (va.size() >= 3) {
                    dir = resolveVec3(parseRef(va[1]));
                    double mag = parseReal(va[2]);
                    dir = dir * mag;
                }
            }
        }
        return std::make_shared<Line3>(origin, dir, 0.0, 1.0);
    }

    std::shared_ptr<ICurve3> resolveCircle(const Entity& rec)
    {
        // CIRCLE('', axis2, radius)
        auto a = splitArgs(rec.body);
        if (a.size() < 3) return nullptr;
        Axis2  ax     = resolveAxis2(parseRef(a[1]));
        double radius = parseReal(a[2]);
        // STEP circle plane: z-axis is normal, x-axis is ref direction for t=0
        // Our Circle3: center, radius, xAxis, yAxis (yAxis = normal x xAxis)
        Vec3 yAxis = ax.zDir.cross(ax.xDir).normalized();
        return std::make_shared<Circle3>(ax.origin, radius, ax.xDir, yAxis);
    }

    std::shared_ptr<ICurve3> resolveBSplineCurve(const Entity& rec)
    {
        // B_SPLINE_CURVE_WITH_KNOTS('',degree,(ctrl),form,closed,self,
        //                           (mults),(knots),knotSpec)
        auto a = splitArgs(rec.body);
        if (a.size() < 8) return nullptr;
        int deg = parseInt(a[1]);
        std::vector<Vec3> ctrl;
        for (int r : parseRefList(a[2])) ctrl.push_back(resolveVec3(r));
        auto mults = parseIntList(a[6]);
        auto knots = parseRealList(a[7]);
        auto flat  = expandKnots(mults, knots);
        try { return std::make_shared<BSplineCurve3>(deg, flat, ctrl); }
        catch (...) {}
        return nullptr;
    }

    std::shared_ptr<ICurve3> resolveQuasiUniformCurve(const Entity& rec)
    {
        // QUASI_UNIFORM_CURVE('', degree, (ctrl), form, closed, self)
        auto a = splitArgs(rec.body);
        if (a.size() < 3) return nullptr;
        int deg = parseInt(a[1]);
        std::vector<Vec3> ctrl;
        for (int r : parseRefList(a[2])) ctrl.push_back(resolveVec3(r));
        if (ctrl.empty()) return nullptr;

        // Degree-1 with 2 points → simple line segment
        if (deg <= 1 && ctrl.size() == 2) {
            Vec3 dir = ctrl[1] - ctrl[0];
            return std::make_shared<Line3>(ctrl[0], dir, 0.0, 1.0);
        }
        // General: build a uniform (clamped) B-spline
        int n = (int)ctrl.size();
        auto flat = BSplineCurve3::uniformKnots(n, deg);
        try { return std::make_shared<BSplineCurve3>(deg, flat, ctrl); }
        catch (...) {}
        // Fallback: line from first to last point
        if (ctrl.size() >= 2) {
            Vec3 dir = ctrl.back() - ctrl.front();
            return std::make_shared<Line3>(ctrl.front(), dir, 0.0, 1.0);
        }
        return nullptr;
    }

    // ── Surface resolvers ─────────────────────────────────────────────────────

    std::shared_ptr<ISurface> resolveSurface(int id)
    {
        auto it = surfCache_.find(id);
        if (it != surfCache_.end()) return it->second;

        const Entity* rec = getEntity(id);
        if (!rec) return nullptr;

        std::shared_ptr<ISurface> result;

        if (rec->complex) {
            result = resolveCompoundSurface(*rec);
        } else if (rec->type == "PLANE") {
            result = resolvePlane(*rec);
        } else if (rec->type == "CYLINDRICAL_SURFACE") {
            result = resolveCylinder(*rec);
        } else if (rec->type == "CONICAL_SURFACE") {
            result = resolveCone(*rec);
        } else if (rec->type == "SPHERICAL_SURFACE") {
            result = resolveSphere(*rec);
        } else if (rec->type == "TOROIDAL_SURFACE") {
            result = resolveTorus(*rec);
        } else if (rec->type == "B_SPLINE_SURFACE_WITH_KNOTS") {
            result = resolveBSplineSurface(*rec);
        }

        surfCache_[id] = result;
        return result;
    }

    std::shared_ptr<ISurface> resolvePlane(const Entity& rec)
    {
        // PLANE('', axis2)
        auto a = splitArgs(rec.body);
        if (a.size() < 2) return nullptr;
        Axis2 ax = resolveAxis2(parseRef(a[1]));
        // In STEP: ax.zDir = plane normal, ax.xDir = u-direction
        Vec3 vAxis = ax.zDir.cross(ax.xDir);
        if (vAxis.squaredNorm() < 1e-20)
            vAxis = ax.xDir.cross(ax.zDir);
        vAxis = vAxis.normalized();
        return std::make_shared<Plane>(ax.origin, ax.xDir, vAxis);
    }

    std::shared_ptr<ISurface> resolveCylinder(const Entity& rec)
    {
        // CYLINDRICAL_SURFACE('', axis2, radius)
        auto a = splitArgs(rec.body);
        if (a.size() < 3) return nullptr;
        Axis2  ax = resolveAxis2(parseRef(a[1]));
        double r  = parseReal(a[2]);
        return std::make_shared<Cylinder>(ax.origin, ax.zDir, r, ax.xDir);
    }

    std::shared_ptr<ISurface> resolveCone(const Entity& rec)
    {
        // CONICAL_SURFACE('', axis2, radius, semi_angle)
        // The placement origin is NOT the apex; radius is radius at origin.
        // apex = origin - radius/tan(semi_angle) * axis
        auto a = splitArgs(rec.body);
        if (a.size() < 4) return nullptr;
        Axis2  ax = resolveAxis2(parseRef(a[1]));
        double r  = parseReal(a[2]);
        double sa = parseReal(a[3]) * angConv_; // convert to radians

        Vec3 apex = ax.origin;
        if (std::abs(r) > 1e-10) {
            double tanA = std::tan(sa);
            if (std::abs(tanA) > 1e-10)
                apex = ax.origin - ax.zDir * (r / tanA);
        }
        return std::make_shared<Cone>(apex, ax.zDir, sa, ax.xDir);
    }

    std::shared_ptr<ISurface> resolveSphere(const Entity& rec)
    {
        // SPHERICAL_SURFACE('', axis2, radius)
        auto a = splitArgs(rec.body);
        if (a.size() < 3) return nullptr;
        Axis2  ax = resolveAxis2(parseRef(a[1]));
        double r  = parseReal(a[2]);
        return std::make_shared<Sphere>(ax.origin, r, ax.zDir, ax.xDir);
    }

    std::shared_ptr<ISurface> resolveTorus(const Entity& rec)
    {
        // TOROIDAL_SURFACE('', axis2, major_radius, minor_radius)
        auto a = splitArgs(rec.body);
        if (a.size() < 4) return nullptr;
        Axis2  ax = resolveAxis2(parseRef(a[1]));
        double R  = parseReal(a[2]);
        double r  = parseReal(a[3]);
        return std::make_shared<Torus>(ax.origin, ax.zDir, R, r, ax.xDir);
    }

    std::shared_ptr<ISurface> resolveBSplineSurface(const Entity& rec)
    {
        // B_SPLINE_SURFACE_WITH_KNOTS('',degU,degV,((ctrl)),form,uClosed,
        //   vClosed,selfIntersect,(uMults),(vMults),(uKnots),(vKnots),knotSpec)
        auto a = splitArgs(rec.body);
        if (a.size() < 12) return nullptr;

        int degU = parseInt(a[1]);
        int degV = parseInt(a[2]);
        auto ctrlGrid = parseRefListList(a[3]);
        // indices: [4]=form,[5]=uClosed,[6]=vClosed,[7]=self
        auto uMults = parseIntList(a[8]);
        auto vMults = parseIntList(a[9]);
        auto uKnots = parseRealList(a[10]);
        auto vKnots = parseRealList(a[11]);

        int nu = (int)ctrlGrid.size();
        int nv = (nu > 0) ? (int)ctrlGrid[0].size() : 0;
        if (nu == 0 || nv == 0) return nullptr;

        BSplineSurface::CtrlGrid pts(nu, std::vector<Vec3>(nv));
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv && j < (int)ctrlGrid[i].size(); ++j)
                pts[i][j] = resolveVec3(ctrlGrid[i][j]);

        auto flatU = expandKnots(uMults, uKnots);
        auto flatV = expandKnots(vMults, vKnots);
        try { return std::make_shared<BSplineSurface>(degU, degV, flatU, flatV, pts); }
        catch (...) {}
        return nullptr;
    }

    std::shared_ptr<ISurface> resolveCompoundSurface(const Entity& rec)
    {
        int degU = 1, degV = 1;
        std::vector<std::vector<int>> ctrlGrid;
        std::vector<int>    uMults, vMults;
        std::vector<double> uKnots, vKnots;
        std::vector<std::vector<double>> weights;
        bool hasKnots = false, hasRational = false;

        for (const auto& part : rec.parts) {
            if (part.type == "B_SPLINE_SURFACE") {
                auto a = splitArgs(part.body);
                if (a.size() >= 3) {
                    degU     = parseInt(a[0]);
                    degV     = parseInt(a[1]);
                    ctrlGrid = parseRefListList(a[2]);
                }
            } else if (part.type == "B_SPLINE_SURFACE_WITH_KNOTS") {
                auto a = splitArgs(part.body);
                if (a.size() >= 4) {
                    uMults = parseIntList(a[0]);
                    vMults = parseIntList(a[1]);
                    uKnots = parseRealList(a[2]);
                    vKnots = parseRealList(a[3]);
                    hasKnots = true;
                }
            } else if (part.type == "RATIONAL_B_SPLINE_SURFACE") {
                auto a = splitArgs(part.body);
                if (!a.empty()) { weights = parseRealListList(a[0]); hasRational = true; }
            }
        }

        if (ctrlGrid.empty() || !hasKnots) return nullptr;

        int nu = (int)ctrlGrid.size();
        int nv = (nu > 0) ? (int)ctrlGrid[0].size() : 0;
        if (nu == 0 || nv == 0) return nullptr;

        BSplineSurface::CtrlGrid pts(nu, std::vector<Vec3>(nv));
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv && j < (int)ctrlGrid[i].size(); ++j)
                pts[i][j] = resolveVec3(ctrlGrid[i][j]);

        auto flatU = expandKnots(uMults, uKnots);
        auto flatV = expandKnots(vMults, vKnots);

        if (hasRational && !weights.empty()) {
            NURBSSurface::WtGrid wt(nu, std::vector<double>(nv, 1.0));
            for (int i = 0; i < nu && i < (int)weights.size(); ++i)
                for (int j = 0; j < nv && j < (int)weights[i].size(); ++j)
                    wt[i][j] = weights[i][j];
            try { return std::make_shared<NURBSSurface>(degU, degV, flatU, flatV, pts, wt); }
            catch (...) {}
        }
        try { return std::make_shared<BSplineSurface>(degU, degV, flatU, flatV, pts); }
        catch (...) {}
        return nullptr;
    }

    // ── Topology resolvers ────────────────────────────────────────────────────

    Handle<Vertex> resolveVertex(int id)
    {
        auto it = vxCache_.find(id);
        if (it != vxCache_.end()) return it->second;

        const Entity* rec = getEntity(id);
        if (!rec || rec->type != "VERTEX_POINT") return {};

        // VERTEX_POINT('', cartesian_point_ref)
        auto a = splitArgs(rec->body);
        if (a.size() < 2) return {};
        Vec3 pt = resolveVec3(parseRef(a[1]));
        auto v  = makeHandle<Vertex>(pt);
        vxCache_[id] = v;
        return v;
    }

    Handle<Edge> resolveEdge(int id)
    {
        auto it = edgeCache_.find(id);
        if (it != edgeCache_.end()) return it->second;

        const Entity* rec = getEntity(id);
        if (!rec || rec->type != "EDGE_CURVE") return {};

        // EDGE_CURVE('', start_vtx, end_vtx, curve, same_sense)
        auto a = splitArgs(rec->body);
        if (a.size() < 5) return {};

        auto sv    = resolveVertex(parseRef(a[1]));
        auto ev    = resolveVertex(parseRef(a[2]));
        auto curve = resolveCurve(parseRef(a[3]));

        auto edge = makeHandle<Edge>(sv, ev);
        if (curve) {
            double tMin = curve->domain().lo;
            double tMax = curve->domain().hi;
            edge->setCurve(curve, tMin, tMax);
        }
        edgeCache_[id] = edge;
        return edge;
    }

    Handle<Wire> resolveWire(int id)
    {
        auto it = wireCache_.find(id);
        if (it != wireCache_.end()) return it->second;

        const Entity* rec = getEntity(id);
        if (!rec || rec->type != "EDGE_LOOP") return {};

        // EDGE_LOOP('', (oriented_edge_refs))
        auto a = splitArgs(rec->body);
        if (a.size() < 2) return {};

        auto wire = makeHandle<Wire>();
        for (int oeId : parseRefList(a[1])) {
            const Entity* oe = getEntity(oeId);
            if (!oe || oe->type != "ORIENTED_EDGE") continue;

            // ORIENTED_EDGE('', *, *, edge_curve_ref, orientation)
            auto oa = splitArgs(oe->body);
            if (oa.size() < 5) continue;
            int edgeRef = parseRef(oa[3]);
            bool fwd    = (trim(oa[4]) == ".T." || trim(oa[4]) == ".TRUE.");

            Handle<Edge> edge = resolveEdge(edgeRef);
            if (!edge) continue;

            CoEdgeOrientation orient =
                fwd ? CoEdgeOrientation::kForward : CoEdgeOrientation::kReversed;
            wire->addCoEdge(makeHandle<CoEdge>(edge, orient));
        }

        wireCache_[id] = wire;
        return wire;
    }

    Handle<Face> resolveFace(int id)
    {
        auto it = faceCache_.find(id);
        if (it != faceCache_.end()) return it->second;

        const Entity* rec = getEntity(id);
        if (!rec || rec->type != "ADVANCED_FACE") return {};

        // ADVANCED_FACE('', (bound_refs), surface_ref, same_sense)
        auto a = splitArgs(rec->body);
        if (a.size() < 4) return {};

        auto face = makeHandle<Face>();

        // Surface
        auto surf = resolveSurface(parseRef(a[2]));
        if (surf) face->setSurface(surf);

        // Orientation
        bool sameDir = (trim(a[3]) == ".T." || trim(a[3]) == ".TRUE.");
        face->setOrientation(sameDir ? FaceOrientation::kForward
                                     : FaceOrientation::kReversed);

        // Bounds: first FACE_OUTER_BOUND → outer wire;
        // fall back to first FACE_BOUND if no FACE_OUTER_BOUND is present.
        bool outerSet = false;
        auto boundRefs = parseRefList(a[1]);

        // Priority pass 1: look for FACE_OUTER_BOUND
        for (int bndRef : boundRefs) {
            const Entity* bnd = getEntity(bndRef);
            if (!bnd || bnd->type != "FACE_OUTER_BOUND") continue;
            auto ba = splitArgs(bnd->body);
            if (ba.size() < 2) continue;
            Handle<Wire> wire = resolveWire(parseRef(ba[1]));
            if (!wire) continue;
            if (!outerSet) { face->setOuterWire(wire); outerSet = true; }
            else           { face->addInnerWire(wire); }
        }
        // Pass 2: FACE_BOUND entries
        if (!outerSet && boundRefs.size() > 1) {
            // AP203 files have only FACE_BOUND.  When multiple bounds are
            // present, find the one with the most coedges – that is the outer
            // boundary; the others are inner holes (e.g. circular cutouts).
            int bestRef  = -1;
            int bestCount = -1;
            for (int bndRef : boundRefs) {
                const Entity* bnd = getEntity(bndRef);
                if (!bnd || bnd->type != "FACE_BOUND") continue;
                auto ba = splitArgs(bnd->body);
                if (ba.size() < 2) continue;
                Handle<Wire> wire = resolveWire(parseRef(ba[1]));
                if (!wire) continue;
                int cnt = (int)wire->coEdges().size();
                if (cnt > bestCount) { bestCount = cnt; bestRef = bndRef; }
            }
            for (int bndRef : boundRefs) {
                const Entity* bnd = getEntity(bndRef);
                if (!bnd || bnd->type != "FACE_BOUND") continue;
                auto ba = splitArgs(bnd->body);
                if (ba.size() < 2) continue;
                Handle<Wire> wire = resolveWire(parseRef(ba[1]));
                if (!wire) continue;
                if (bndRef == bestRef) { face->setOuterWire(wire); outerSet = true; }
                else                  { face->addInnerWire(wire); }
            }
        } else {
            for (int bndRef : boundRefs) {
                const Entity* bnd = getEntity(bndRef);
                if (!bnd || bnd->type != "FACE_BOUND") continue;
                auto ba = splitArgs(bnd->body);
                if (ba.size() < 2) continue;
                Handle<Wire> wire = resolveWire(parseRef(ba[1]));
                if (!wire) continue;
                if (!outerSet) { face->setOuterWire(wire); outerSet = true; }
                else           { face->addInnerWire(wire); }
            }
        }

        faceCache_[id] = face;
        return face;
    }

    Handle<Shell> resolveShell(int id)
    {
        auto it = shellCache_.find(id);
        if (it != shellCache_.end()) return it->second;

        const Entity* rec = getEntity(id);
        if (!rec) return {};
        bool closed = (rec->type == "CLOSED_SHELL");
        bool open   = (rec->type == "OPEN_SHELL");
        if (!closed && !open) return {};

        // CLOSED_SHELL / OPEN_SHELL ('', (face_refs))
        auto a = splitArgs(rec->body);
        if (a.size() < 2) return {};

        auto shell = makeHandle<Shell>();
        shell->setClosed(closed);
        for (int fId : parseRefList(a[1])) {
            auto face = resolveFace(fId);
            if (face) shell->addFace(face);
        }

        shellCache_[id] = shell;
        return shell;
    }

    // ── Body building ─────────────────────────────────────────────────────────

    std::vector<Handle<Body>> buildBodies()
    {
        std::vector<Handle<Body>> result;

        // Collect MANIFOLD_SOLID_BREP IDs referenced from any
        // ADVANCED_BREP_SHAPE_REPRESENTATION. Fall back to all BREP entities.
        std::vector<int> brepIds;

        for (auto& kv : entities_) {
            if (kv.second.type == "ADVANCED_BREP_SHAPE_REPRESENTATION") {
                auto a = splitArgs(kv.second.body);
                if (a.size() >= 2) {
                    for (int r : parseRefList(a[1])) {
                        const Entity* e = getEntity(r);
                        if (e && e->type == "MANIFOLD_SOLID_BREP")
                            brepIds.push_back(r);
                    }
                }
            }
        }

        // Fallback: collect all MANIFOLD_SOLID_BREP entities
        if (brepIds.empty()) {
            for (auto& kv : entities_) {
                if (kv.second.type == "MANIFOLD_SOLID_BREP")
                    brepIds.push_back(kv.first);
            }
        }

        if (brepIds.empty()) return result;

        // Build one Body containing one Lump per MANIFOLD_SOLID_BREP
        auto body = makeHandle<Body>();
        for (int brepId : brepIds) {
            const Entity* rec = getEntity(brepId);
            if (!rec || rec->type != "MANIFOLD_SOLID_BREP") continue;

            // MANIFOLD_SOLID_BREP('', shell_ref)
            auto a = splitArgs(rec->body);
            if (a.size() < 2) continue;

            auto shell = resolveShell(parseRef(a[1]));
            if (!shell) continue;

            auto lump = makeHandle<Lump>();
            lump->setOuterShell(shell);
            body->addLump(lump);
        }

        if (!body->lumps().empty())
            result.push_back(body);

        return result;
    }
};

} // namespace gk
