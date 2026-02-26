// Chapter 9.2 — STEP export: complex mechanical model tests.
//
// Generates 30 STEP files of realistic mechanical components using the
// geometric kernel's primitive builder functions (makeBox, makeSphere,
// makeCylinder, makeCone, makeTorus) and parametric surface/curve types.
//
// Every test:
//  1. Constructs a multi-solid or multi-shape B-Rep body.
//  2. Exports it via StepWriter::write().
//  3. Validates structural STEP correctness (no empty EDGE_LOOP, required
//     entities present, non-empty file).
//  4. Writes the .step file to step_test_output/ for CAD Assistant validation.

#include "gk/brep/BRep.h"
#include "gk/builders/PrimitiveBuilders.h"
#include "gk/builders/MoreBuilders.h"
#include "gk/surface/BSplineSurface.h"
#include "gk/surface/NURBSSurface.h"
#include "gk/surface/RuledSurface.h"
#include "gk/surface/Cylinder.h"
#include "gk/surface/Cone.h"
#include "gk/surface/Sphere.h"
#include "gk/surface/Torus.h"
#include "gk/surface/DiscSurface.h"
#include "gk/surface/Plane.h"
#include "gk/curve/Line.h"
#include "gk/curve/Circle.h"
#include "gk/curve/BSplineCurve.h"
#include "gk/curve/NURBSCurve.h"
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

using namespace gk;
static constexpr double kPi = 3.14159265358979323846;

// ── Shared helpers ─────────────────────────────────────────────────────────────

static std::string stepOutputPath(const std::string& filename)
{
    std::filesystem::create_directories("step_test_output");
    return "step_test_output/" + filename;
}

static bool contains(const std::string& h, const std::string& n)
{
    return h.find(n) != std::string::npos;
}

static void assertNoEmptyEdgeLoops(const std::string& step)
{
    EXPECT_EQ(step.find("EDGE_LOOP('',())"), std::string::npos)
        << "Found empty EDGE_LOOP('',()) — face boundary is missing";
}

static void assertValidStep(const std::string& step)
{
    EXPECT_FALSE(step.empty());
    EXPECT_TRUE(contains(step, "ISO-10303-21;"));
    EXPECT_TRUE(contains(step, "END-ISO-10303-21;"));
    EXPECT_TRUE(contains(step, "ADVANCED_BREP_SHAPE_REPRESENTATION"));
    assertNoEmptyEdgeLoops(step);
}

/// Write step to a file and return whether it succeeded.
static bool writeToFile(const std::string& step, const std::string& filename)
{
    std::ofstream f(stepOutputPath(filename));
    if (!f.is_open()) return false;
    f << step;
    return f.good();
}

/// Combine multiple primitive bodies into one by merging their lumps.
static Handle<Body> combineBodies(std::initializer_list<Handle<Body>> bodies)
{
    auto result = makeHandle<Body>();
    for (auto& b : bodies)
        for (auto& lump : b->lumps())
            result->addLump(lump);
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// 1.  Cylindrical pin / dowel
// ─────────────────────────────────────────────────────────────────────────────
// Slender cylinder: r=2 mm, h=20 mm
TEST(StepModels, Pin_Cylinder)
{
    auto body = makeCylinder(Vec3::zero(), Vec3::unitZ(), 2.0, 20.0);
    auto step = StepWriter::write(*body, "Pin");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "pin.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 2.  Hex bolt  (shaft + hex-head modelled as wide short cylinder)
// ─────────────────────────────────────────────────────────────────────────────
// Shaft: r=3, h=25;  Head: r=6, h=4
TEST(StepModels, Bolt_ShaftAndHead)
{
    auto shaft = makeCylinder(Vec3{0, 0, 0},  Vec3::unitZ(), 3.0, 25.0);
    auto head  = makeCylinder(Vec3{0, 0, -4}, Vec3::unitZ(), 6.0,  4.0);
    auto body  = combineBodies({shaft, head});
    auto step  = StepWriter::write(*body, "Bolt");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "bolt.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 3.  Countersunk screw  (shaft + flat head modelled as shallow cone)
// ─────────────────────────────────────────────────────────────────────────────
// Shaft: r=2, h=20;  Head: cone apex at z=0, half-angle=45°, h=4
TEST(StepModels, Screw_CountersunkHead)
{
    auto shaft = makeCylinder(Vec3{0, 0, 0}, Vec3::unitZ(), 2.0, 20.0);
    // 90° countersink cone (half-angle 45°) — apex at origin, opens downward
    auto head  = makeCone(Vec3{0, 0, 0}, Vec3{0, 0, -1}, kPi / 4.0, 4.0);
    auto body  = combineBodies({shaft, head});
    auto step  = StepWriter::write(*body, "CountersunkScrew");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CONICAL_SURFACE"));
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "screw_countersunk.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 4.  Stepped shaft  (three coaxial cylinders of decreasing diameter)
// ─────────────────────────────────────────────────────────────────────────────
// Section A: r=8, h=10;  Section B: r=5, h=20;  Section C: r=3, h=15
TEST(StepModels, SteppedShaft)
{
    auto secA = makeCylinder(Vec3{0, 0, 0},  Vec3::unitZ(), 8.0, 10.0);
    auto secB = makeCylinder(Vec3{0, 0, 10}, Vec3::unitZ(), 5.0, 20.0);
    auto secC = makeCylinder(Vec3{0, 0, 30}, Vec3::unitZ(), 3.0, 15.0);
    auto body = combineBodies({secA, secB, secC});
    auto step = StepWriter::write(*body, "SteppedShaft");
    assertValidStep(step);
    EXPECT_TRUE(writeToFile(step, "stepped_shaft.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 5.  Flat washer  (very flat short cylinder, large radius)
// ─────────────────────────────────────────────────────────────────────────────
// r=12, h=1.5
TEST(StepModels, Washer_Flat)
{
    auto body = makeCylinder(Vec3::zero(), Vec3::unitZ(), 12.0, 1.5);
    auto step = StepWriter::write(*body, "Washer");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "washer.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 6.  O-ring / gasket  (thin small torus)
// ─────────────────────────────────────────────────────────────────────────────
// majorR=15, minorR=1.5
TEST(StepModels, ORing_Gasket)
{
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), 15.0, 1.5);
    auto step = StepWriter::write(*body, "ORing");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "TOROIDAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "oring.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 7.  Automotive tire profile  (fat torus)
// ─────────────────────────────────────────────────────────────────────────────
// majorR=280, minorR=95  (mm scale, bicycle tire)
TEST(StepModels, Torus_TireProfile)
{
    auto body = makeTorus(Vec3::zero(), Vec3::unitZ(), 280.0, 95.0);
    auto step = StepWriter::write(*body, "TireProfile");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "TOROIDAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "tire_profile.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 8.  Ball bearing assembly  (outer ring + 6 balls arranged on a pitch circle)
// ─────────────────────────────────────────────────────────────────────────────
// Ring (torus): majorR=20, minorR=3;  Balls: r=3, on r=20 pitch circle
TEST(StepModels, BallBearing_Assembly)
{
    std::vector<Handle<Body>> parts;

    // Bearing ring (torus)
    parts.push_back(makeTorus(Vec3::zero(), Vec3::unitZ(), 20.0, 3.0));

    // 6 ball spheres evenly spaced on the pitch circle
    for (int i = 0; i < 6; ++i) {
        double angle = 2.0 * kPi * double(i) / 6.0;
        Vec3 center{20.0 * std::cos(angle), 20.0 * std::sin(angle), 0.0};
        parts.push_back(makeSphere(center, 3.0));
    }

    auto body = makeHandle<Body>();
    for (auto& p : parts)
        for (auto& l : p->lumps())
            body->addLump(l);

    auto step = StepWriter::write(*body, "BallBearing");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "TOROIDAL_SURFACE"));
    EXPECT_TRUE(contains(step, "SPHERICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "ball_bearing.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 9.  Dome cap  (sphere on a short cylindrical pedestal)
// ─────────────────────────────────────────────────────────────────────────────
// Sphere center at (0,0,20), r=10;  Pedestal: cylinder r=10, h=20
TEST(StepModels, DomeCap_SphereOnCylinder)
{
    auto pedestal = makeCylinder(Vec3{0, 0, 0}, Vec3::unitZ(), 10.0, 20.0);
    auto dome     = makeSphere(Vec3{0, 0, 20}, 10.0);
    auto body     = combineBodies({pedestal, dome});
    auto step     = StepWriter::write(*body, "DomeCap");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "SPHERICAL_SURFACE"));
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "dome_cap.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. Tapered plug / taper pin  (solid cone frustum)
// ─────────────────────────────────────────────────────────────────────────────
// Cone: halfAngle=5°, apex at origin, height=30
TEST(StepModels, TaperedPlug)
{
    auto body = makeCone(Vec3::zero(), Vec3::unitZ(), 5.0 * kPi / 180.0, 30.0);
    auto step = StepWriter::write(*body, "TaperedPlug");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CONICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "tapered_plug.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 11. Drill bit tip  (narrow cone, halfAngle=59° ≈ standard drill geometry)
// ─────────────────────────────────────────────────────────────────────────────
TEST(StepModels, DrillBitTip)
{
    // Standard 118° included angle → half-angle = 59°
    auto tip = makeCone(Vec3::zero(), Vec3::unitZ(), 59.0 * kPi / 180.0, 6.0);
    auto shank = makeCylinder(Vec3{0, 0, 6}, Vec3::unitZ(), 3.0, 50.0);
    auto body  = combineBodies({tip, shank});
    auto step  = StepWriter::write(*body, "DrillBitTip");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CONICAL_SURFACE"));
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "drill_bit_tip.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 12. Funnel  (wide-angle cone, halfAngle=70°)
// ─────────────────────────────────────────────────────────────────────────────
TEST(StepModels, Funnel_WideCone)
{
    auto cone = makeCone(Vec3::zero(), Vec3::unitZ(), 70.0 * kPi / 180.0, 25.0);
    auto body = combineBodies({cone});
    auto step = StepWriter::write(*body, "Funnel");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CONICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "funnel.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 13. Rivet  (cylinder body + spherical dome head)
// ─────────────────────────────────────────────────────────────────────────────
// Shaft: r=3, h=12;  Dome: sphere r=4 centred on top of shaft
TEST(StepModels, Rivet_CylinderAndDome)
{
    auto shaft = makeCylinder(Vec3{0, 0, 0}, Vec3::unitZ(), 3.0, 12.0);
    auto dome  = makeSphere(Vec3{0, 0, 12}, 4.0);
    auto body  = combineBodies({shaft, dome});
    auto step  = StepWriter::write(*body, "Rivet");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "SPHERICAL_SURFACE"));
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "rivet.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 14. Spool body  (two wide flanges + narrow hub)
// ─────────────────────────────────────────────────────────────────────────────
// Flange1: r=20, h=3;  Hub: r=8, h=30;  Flange2: r=20, h=3  (total h=36)
TEST(StepModels, SpoolBody)
{
    auto fl1  = makeCylinder(Vec3{0, 0, 0},  Vec3::unitZ(), 20.0,  3.0);
    auto hub  = makeCylinder(Vec3{0, 0, 3},  Vec3::unitZ(),  8.0, 30.0);
    auto fl2  = makeCylinder(Vec3{0, 0, 33}, Vec3::unitZ(), 20.0,  3.0);
    auto body = combineBodies({fl1, hub, fl2});
    auto step = StepWriter::write(*body, "Spool");
    assertValidStep(step);
    EXPECT_TRUE(writeToFile(step, "spool.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 15. Nozzle  (cylinder inlet + cone taper + small cylinder outlet)
// ─────────────────────────────────────────────────────────────────────────────
// Inlet: r=15, h=10;  Taper cone: halfAngle=25°, h=20;  Outlet: r=8, h=15
TEST(StepModels, Nozzle_CylinderConeAssembly)
{
    auto inlet   = makeCylinder(Vec3{0, 0, 0},  Vec3::unitZ(), 15.0, 10.0);
    // Cone taper: we approximate with a wide cone from apex=z=10+20 opening downward
    auto taper   = makeCone(Vec3{0, 0, 30}, Vec3{0,0,-1}, 25.0 * kPi / 180.0, 20.0);
    auto outlet  = makeCylinder(Vec3{0, 0, -5}, Vec3::unitZ(),  8.0, 15.0);
    auto body    = combineBodies({inlet, taper, outlet});
    auto step    = StepWriter::write(*body, "Nozzle");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CONICAL_SURFACE"));
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "nozzle.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 16. I-beam cross-section approximation  (three boxes: top/bottom flanges + web)
// ─────────────────────────────────────────────────────────────────────────────
// Web: 4×100×60mm;  Top flange: 40×100×8mm;  Bottom flange: 40×100×8mm
TEST(StepModels, IBeam_ThreeBoxes)
{
    // Web (centre): x ∈ [-2,2], y ∈ [0,100], z ∈ [0,60]
    auto web    = makeBox(Vec3{-2, 0, 0},  Vec3{2, 100, 60});
    // Top flange: x ∈ [-20,20], y ∈ [0,100], z ∈ [60,68]
    auto topFl  = makeBox(Vec3{-20, 0, 60}, Vec3{20, 100, 68});
    // Bottom flange: x ∈ [-20,20], y ∈ [0,100], z ∈ [-8,0]
    auto botFl  = makeBox(Vec3{-20, 0, -8}, Vec3{20, 100, 0});
    auto body   = combineBodies({web, topFl, botFl});
    auto step   = StepWriter::write(*body, "IBeam");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "PLANE("));
    EXPECT_TRUE(writeToFile(step, "ibeam.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 17. L-bracket  (two flat boxes joined at 90°)
// ─────────────────────────────────────────────────────────────────────────────
// Horizontal arm: 80×10×5mm;  Vertical arm: 5×10×50mm
TEST(StepModels, LBracket_TwoBoxes)
{
    auto horiz = makeBox(Vec3{0, 0, 0},  Vec3{80, 10, 5});
    auto vert  = makeBox(Vec3{0, 0, 5},  Vec3{ 5, 10, 55});
    auto body  = combineBodies({horiz, vert});
    auto step  = StepWriter::write(*body, "LBracket");
    assertValidStep(step);
    EXPECT_TRUE(writeToFile(step, "l_bracket.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 18. Cross brace  (four thin long boxes in a '+' pattern)
// ─────────────────────────────────────────────────────────────────────────────
// Each arm: 60×5×5mm; arms oriented along X and Y from centre
TEST(StepModels, CrossBrace)
{
    auto armX1 = makeBox(Vec3{  0, -2.5, 0}, Vec3{60, 2.5, 5});
    auto armX2 = makeBox(Vec3{-60, -2.5, 0}, Vec3{ 0, 2.5, 5});
    auto armY1 = makeBox(Vec3{-2.5,   0, 0}, Vec3{2.5, 60, 5});
    auto armY2 = makeBox(Vec3{-2.5, -60, 0}, Vec3{2.5,  0, 5});
    auto body  = combineBodies({armX1, armX2, armY1, armY2});
    auto step  = StepWriter::write(*body, "CrossBrace");
    assertValidStep(step);
    EXPECT_TRUE(writeToFile(step, "cross_brace.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 19. Pipe tee junction  (three cylinders at right angles)
// ─────────────────────────────────────────────────────────────────────────────
// Main pipe (Z-axis): r=5, h=40;  Branch (X-axis): r=5, h=20
TEST(StepModels, PipeTee_ThreeCylinders)
{
    auto mainPipe   = makeCylinder(Vec3{0, 0, -20}, Vec3::unitZ(),  5.0, 40.0);
    auto branchPos  = makeCylinder(Vec3{0, 0,  0},  Vec3::unitX(),  5.0, 20.0);
    auto branchNeg  = makeCylinder(Vec3{0, 0,  0},  Vec3{-1,0,0},  5.0, 20.0);
    auto body       = combineBodies({mainPipe, branchPos, branchNeg});
    auto step       = StepWriter::write(*body, "PipeTee");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "pipe_tee.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 20. Gear-blank (cylinder) + shaft hole placeholder (inner cylinder)
// ─────────────────────────────────────────────────────────────────────────────
// Both exported as separate solids (no boolean subtract available yet)
// Gear blank: r=40, h=10;  Shaft cylinder (key): r=6, h=15
TEST(StepModels, GearBlank_WithShaftCylinder)
{
    auto blank = makeCylinder(Vec3{0, 0, 0}, Vec3::unitZ(), 40.0, 10.0);
    auto shaft = makeCylinder(Vec3{0, 0, 0}, Vec3::unitZ(),  6.0, 15.0);
    auto body  = combineBodies({blank, shaft});
    auto step  = StepWriter::write(*body, "GearBlank");
    assertValidStep(step);
    EXPECT_TRUE(writeToFile(step, "gear_blank.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 21. Cam disc  (cylinder with tilted axis — eccentric cam geometry)
// ─────────────────────────────────────────────────────────────────────────────
// Axis tilted 30° from Z;  r=20, h=8
TEST(StepModels, CamDisc_TiltedAxis)
{
    Vec3 tiltAxis = Vec3{std::sin(30.0*kPi/180.0), 0.0, std::cos(30.0*kPi/180.0)};
    auto body = makeCylinder(Vec3::zero(), tiltAxis, 20.0, 8.0);
    auto step = StepWriter::write(*body, "CamDisc");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "cam_disc.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 22. Compound sphere array  (3×3 grid of spheres, like a ball array)
// ─────────────────────────────────────────────────────────────────────────────
TEST(StepModels, SphereArray_3x3)
{
    std::vector<Handle<Body>> balls;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            balls.push_back(makeSphere(Vec3{double(i)*15, double(j)*15, 0}, 5.0));

    auto body = makeHandle<Body>();
    for (auto& b : balls)
        for (auto& l : b->lumps())
            body->addLump(l);

    auto step = StepWriter::write(*body, "SphereArray");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "SPHERICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "sphere_array.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 23. Offset box assembly  (plate + four corner posts)
// ─────────────────────────────────────────────────────────────────────────────
// Base plate: 100×80×5mm;  Four cylindrical posts: r=4, h=30 at each corner
TEST(StepModels, OffsetBoxAssembly_PlateAndPosts)
{
    auto plate = makeBox(Vec3{0, 0, 0}, Vec3{100, 80, 5});
    // Posts at 4 inner corners
    auto p1 = makeCylinder(Vec3{ 5,  5, 5}, Vec3::unitZ(), 4.0, 30.0);
    auto p2 = makeCylinder(Vec3{95,  5, 5}, Vec3::unitZ(), 4.0, 30.0);
    auto p3 = makeCylinder(Vec3{ 5, 75, 5}, Vec3::unitZ(), 4.0, 30.0);
    auto p4 = makeCylinder(Vec3{95, 75, 5}, Vec3::unitZ(), 4.0, 30.0);
    auto body = combineBodies({plate, p1, p2, p3, p4});
    auto step = StepWriter::write(*body, "PlateWithPosts");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
    EXPECT_TRUE(contains(step, "PLANE("));
    EXPECT_TRUE(writeToFile(step, "plate_with_posts.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 24. BSpline freeform curved panel  (4×4 control mesh, degree 3)
// ─────────────────────────────────────────────────────────────────────────────
// A saddle-shaped surface patch
TEST(StepModels, BSpline_CurvedPanel)
{
    int nu = 4, nv = 4;
    BSplineSurface::CtrlGrid ctrl(nu, std::vector<Vec3>(nv));
    // Create a gentle saddle shape: z = 0.1 * (x² − y²)
    for (int i = 0; i < nu; ++i) {
        for (int j = 0; j < nv; ++j) {
            double u = double(i) * 10.0;  // x ∈ [0, 30]
            double v = double(j) * 10.0;  // y ∈ [0, 30]
            double z = 0.02 * (u*u - v*v); // saddle height
            ctrl[i][j] = Vec3{u, v, z};
        }
    }
    auto kU = BSplineSurface::uniformKnots(nu, 3);
    auto kV = BSplineSurface::uniformKnots(nv, 3);
    auto surf = std::make_shared<BSplineSurface>(3, 3, kU, kV, ctrl);

    auto face = makeHandle<Face>();
    face->setSurface(surf);
    face->setOuterWire(makeHandle<Wire>());
    face->setUVDomain(surf->domain());

    auto shell = makeHandle<Shell>();
    shell->setClosed(false);
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);
    auto body = makeHandle<Body>();
    body->addLump(lump);

    auto step = StepWriter::write(*body, "BSplineSaddlePanel");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "B_SPLINE_SURFACE_WITH_KNOTS"));
    EXPECT_TRUE(writeToFile(step, "bspline_saddle_panel.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 25. NURBS free-form surface patch  (3×3 control mesh, degree 2)
// ─────────────────────────────────────────────────────────────────────────────
// A smooth bowl/dome surface with non-uniform rational weights
TEST(StepModels, NURBS_SphericalPatch)
{
    // 3×3 bilinear-to-quadratic NURBS patch forming a curved bowl:
    //   control points span a 20×20mm square with varying heights
    double w = std::sqrt(2.0) / 2.0; // ≈ 0.7071  (standard conical weight)

    NURBSSurface::CtrlGrid ctrl = {
        { Vec3{ 0,  0,  0}, Vec3{ 0, 10,  5}, Vec3{ 0, 20,  0} },
        { Vec3{10,  0,  5}, Vec3{10, 10, 12}, Vec3{10, 20,  5} },
        { Vec3{20,  0,  0}, Vec3{20, 10,  5}, Vec3{20, 20,  0} }
    };
    NURBSSurface::WtGrid wt = {
        { 1.0, w,   1.0 },
        { w,   0.5, w   },
        { 1.0, w,   1.0 }
    };

    int nu = 3, nv = 3;
    auto kU = BSplineSurface::uniformKnots(nu, 2);
    auto kV = BSplineSurface::uniformKnots(nv, 2);
    auto surf = std::make_shared<NURBSSurface>(2, 2, kU, kV, ctrl, wt);

    auto face = makeHandle<Face>();
    face->setSurface(surf);
    face->setOuterWire(makeHandle<Wire>());
    face->setUVDomain(surf->domain());

    auto shell = makeHandle<Shell>();
    shell->setClosed(false);
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);
    auto body = makeHandle<Body>();
    body->addLump(lump);

    auto step = StepWriter::write(*body, "NURBSBowlPatch");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "RATIONAL_B_SPLINE_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "nurbs_spherical_patch.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 26. Ruled surface  (linear loft between two parallel lines)
// ─────────────────────────────────────────────────────────────────────────────
// Tapered flat panel: left edge at x=0, right edge at x=30; z varies from 0 to 10
TEST(StepModels, RuledSurface_TaperedPanel)
{
    // Bottom edge: Line3 from (0,0,0) to (30,0,0)
    auto line0 = std::make_shared<Line3>(Vec3{0,0,0}, Vec3{1,0,0}, 0.0, 30.0);
    // Top edge: Line3 from (0,20,10) to (30,20,10) (offset + raised)
    auto line1 = std::make_shared<Line3>(Vec3{0,20,10}, Vec3{1,0,0}, 0.0, 30.0);
    auto surf = std::make_shared<RuledSurface>(line0, line1);

    auto face = makeHandle<Face>();
    face->setSurface(surf);
    face->setOuterWire(makeHandle<Wire>());
    face->setUVDomain(surf->domain());

    auto shell = makeHandle<Shell>();
    shell->setClosed(false);
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);
    auto body = makeHandle<Body>();
    body->addLump(lump);

    // StepWriter falls back to PLANE for unknown surfaces — verify it doesn't crash
    auto step = StepWriter::write(*body, "RuledSurfacePanel");
    EXPECT_FALSE(step.empty());
    EXPECT_TRUE(contains(step, "ISO-10303-21;"));
    EXPECT_TRUE(writeToFile(step, "ruled_surface_panel.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 27. Large box (structural beam) + two mounting spheres at each end
// ─────────────────────────────────────────────────────────────────────────────
// Beam: 200×20×20mm;  Mounting spheres r=12 at (0,10,10) and (200,10,10)
TEST(StepModels, StructuralBeam_WithMountings)
{
    auto beam = makeBox(Vec3{0, 0, 0}, Vec3{200, 20, 20});
    auto mntA = makeSphere(Vec3{  0, 10, 10}, 12.0);
    auto mntB = makeSphere(Vec3{200, 10, 10}, 12.0);
    auto body = combineBodies({beam, mntA, mntB});
    auto step = StepWriter::write(*body, "StructuralBeam");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "SPHERICAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "structural_beam.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 28. Rack-and-pinion placeholder
//     Rack = thin flat box; Pinion = cylinder on its side
// ─────────────────────────────────────────────────────────────────────────────
TEST(StepModels, RackAndPinion_Placeholder)
{
    // Rack: 120×10×5mm
    auto rack = makeBox(Vec3{0, 0, 0}, Vec3{120, 10, 5});
    // Pinion: cylinder with axis along Y, r=15, h=10, positioned above rack
    auto pinion = makeCylinder(Vec3{60, 0, 20}, Vec3::unitY(), 15.0, 10.0);
    auto body = combineBodies({rack, pinion});
    auto step = StepWriter::write(*body, "RackAndPinion");
    assertValidStep(step);
    EXPECT_TRUE(writeToFile(step, "rack_and_pinion.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 29. Turbine blade placeholder
//     Airfoil profile approximated with a BSpline surface lofted between
//     leading-edge and trailing-edge control spans
// ─────────────────────────────────────────────────────────────────────────────
TEST(StepModels, TurbineBlade_BSpline)
{
    // 5×3 BSpline surface (degree 3 in span, degree 2 in chord)
    // Represents a twisted, cambered blade surface
    int nu = 5, nv = 3;
    BSplineSurface::CtrlGrid ctrl(nu, std::vector<Vec3>(nv));
    for (int i = 0; i < nu; ++i) {
        double span  = double(i) * 10.0;  // z span [0, 40]
        double twist = span * 0.04;        // gentle twist angle (rad)
        // Chord control points: leading, middle, trailing
        double chord[3] = {0.0, 5.0, 10.0};
        for (int j = 0; j < nv; ++j) {
            double x = chord[j] * std::cos(twist);
            double y = chord[j] * std::sin(twist) + (j == 1 ? 2.0 : 0.0); // camber
            ctrl[i][j] = Vec3{x, y, span};
        }
    }
    auto kU = BSplineSurface::uniformKnots(nu, 3);
    auto kV = BSplineSurface::uniformKnots(nv, 2);
    auto surf = std::make_shared<BSplineSurface>(3, 2, kU, kV, ctrl);

    auto face = makeHandle<Face>();
    face->setSurface(surf);
    face->setOuterWire(makeHandle<Wire>());
    face->setUVDomain(surf->domain());

    auto shell = makeHandle<Shell>();
    shell->setClosed(false);
    shell->addFace(face);

    auto lump = makeHandle<Lump>();
    lump->setOuterShell(shell);
    auto body = makeHandle<Body>();
    body->addLump(lump);

    auto step = StepWriter::write(*body, "TurbineBlade");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "B_SPLINE_SURFACE_WITH_KNOTS"));
    EXPECT_TRUE(writeToFile(step, "turbine_blade.step"));
}

// ─────────────────────────────────────────────────────────────────────────────
// 30. Spring (torus approximation of a helical coil cross-section)
//     Modelled as a chain of 6 small tori arranged in a circle
// ─────────────────────────────────────────────────────────────────────────────
TEST(StepModels, Spring_TorusChain)
{
    // 6 small tori arranged as if coiled: each torus at angle on a pitch circle
    // This approximates one turn of a spring, viewed from above
    double pitchR = 25.0;   // coil radius
    double wireR  =  2.5;   // wire cross-section

    std::vector<Handle<Body>> coils;
    for (int i = 0; i < 6; ++i) {
        double angle = 2.0 * kPi * double(i) / 6.0;
        Vec3 center{pitchR * std::cos(angle), pitchR * std::sin(angle), 0.0};
        // Each mini-torus has its axis pointing tangentially along the coil
        Vec3 tangent{-std::sin(angle), std::cos(angle), 0.1}; // slight helix pitch
        coils.push_back(makeTorus(center, tangent.normalized(), pitchR * 0.3, wireR));
    }

    auto body = makeHandle<Body>();
    for (auto& c : coils)
        for (auto& l : c->lumps())
            body->addLump(l);

    auto step = StepWriter::write(*body, "SpringCoil");
    assertValidStep(step);
    EXPECT_TRUE(contains(step, "TOROIDAL_SURFACE"));
    EXPECT_TRUE(writeToFile(step, "spring_coil.step"));
}

} // namespace
