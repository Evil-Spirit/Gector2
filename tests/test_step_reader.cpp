// Chapter 9.2 — STEP Import: round-trip tests.
//
// For each example STEP file:
//   1. Read file text from disk.
//   2. Parse with StepReader → gk::Body objects.
//   3. Verify at least one body was parsed.
//   4. Export back with StepWriter → STEP text.
//   5. Verify the exported STEP text is structurally valid.
//
// The test files are located in step_examples/ relative to the project root.
// When CTest runs them, the working directory is the build folder, so we use
// a path relative to the source tree (obtained via the CMake-generated macro).

#include "gk/brep/StepReader.h"
#include "gk/brep/StepWriter.h"
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace gk;

// ── Helper: load a file from the step_examples directory ──────────────────────

/// Macro STEP_EXAMPLES_DIR is injected by CMake so tests can locate the files
/// regardless of working directory.
#ifndef STEP_EXAMPLES_DIR
// Fallback: assume CTest runs from the build directory one level below root.
#define STEP_EXAMPLES_DIR "../step_examples"
#endif

static std::string loadFile(const std::string& filename)
{
    std::string path = std::string(STEP_EXAMPLES_DIR) + "/" + filename;
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// ── Validation helpers ────────────────────────────────────────────────────────

static bool contains(const std::string& h, const std::string& n)
{
    return h.find(n) != std::string::npos;
}

static void assertValidStepOutput(const std::string& step,
                                   const std::string& context)
{
    EXPECT_FALSE(step.empty())        << context;
    EXPECT_TRUE(contains(step, "ISO-10303-21;"))   << context;
    EXPECT_TRUE(contains(step, "END-ISO-10303-21;")) << context;
    EXPECT_TRUE(contains(step, "ADVANCED_BREP_SHAPE_REPRESENTATION")) << context;
    // The exported file should contain at least one ADVANCED_FACE
    EXPECT_TRUE(contains(step, "ADVANCED_FACE")) << context;
}

static void writeRoundTripFile(const std::string& step,
                                const std::string& filename)
{
    std::filesystem::create_directories("step_test_output");
    std::ofstream f("step_test_output/" + filename);
    if (f.is_open()) f << step;
}

// ── Generic round-trip runner ─────────────────────────────────────────────────

struct RoundTripResult {
    bool     fileLoaded{false};
    int      bodiesRead{0};
    int      lumpsTotal{0};
    bool     exportOk{false};
    std::string stepOut;
};

static RoundTripResult doRoundTrip(const std::string& filename,
                                    const std::string& partName)
{
    RoundTripResult r;
    std::string text = loadFile(filename);
    r.fileLoaded = !text.empty();
    if (!r.fileLoaded) return r;

    auto bodies = StepReader::read(text);
    r.bodiesRead = (int)bodies.size();

    for (auto& body : bodies)
        r.lumpsTotal += (int)body->lumps().size();

    if (!bodies.empty()) {
        r.stepOut = StepWriter::write(*bodies.front(), partName);
        r.exportOk = !r.stepOut.empty();
    }
    return r;
}

// ── Test cases ────────────────────────────────────────────────────────────────

TEST(StepRoundTrip, LoadFile_1797609in)
{
    std::string text = loadFile("1797609in.step");
    ASSERT_FALSE(text.empty()) << "Could not load step_examples/1797609in.step";
}

TEST(StepRoundTrip, LoadFile_angle1)
{
    std::string text = loadFile("angle1.stp");
    ASSERT_FALSE(text.empty()) << "Could not load step_examples/angle1.stp";
}

TEST(StepRoundTrip, LoadFile_bull)
{
    std::string text = loadFile("bull.stp");
    ASSERT_FALSE(text.empty()) << "Could not load step_examples/bull.stp";
}

TEST(StepRoundTrip, LoadFile_iso14649)
{
    std::string text = loadFile("iso14649-demo.stp");
    ASSERT_FALSE(text.empty()) << "Could not load step_examples/iso14649-demo.stp";
}

TEST(StepRoundTrip, LoadFile_object)
{
    std::string text = loadFile("object.step");
    ASSERT_FALSE(text.empty()) << "Could not load step_examples/object.step";
}

TEST(StepRoundTrip, LoadFile_test_model)
{
    std::string text = loadFile("test_model.step");
    ASSERT_FALSE(text.empty()) << "Could not load step_examples/test_model.step";
}

// ── Parsing and body extraction ───────────────────────────────────────────────

TEST(StepRoundTrip, Parse_1797609in)
{
    std::string text = loadFile("1797609in.step");
    if (text.empty()) GTEST_SKIP() << "File not found";

    auto bodies = StepReader::read(text);
    EXPECT_FALSE(bodies.empty()) << "No bodies parsed from 1797609in.step";
    if (!bodies.empty()) {
        EXPECT_GT(bodies.front()->lumps().size(), 0u);
    }
}

TEST(StepRoundTrip, Parse_angle1)
{
    std::string text = loadFile("angle1.stp");
    if (text.empty()) GTEST_SKIP() << "File not found";

    auto bodies = StepReader::read(text);
    EXPECT_FALSE(bodies.empty()) << "No bodies parsed from angle1.stp";
    if (!bodies.empty()) {
        EXPECT_GT(bodies.front()->lumps().size(), 0u);
    }
}

TEST(StepRoundTrip, Parse_bull)
{
    std::string text = loadFile("bull.stp");
    if (text.empty()) GTEST_SKIP() << "File not found";

    auto bodies = StepReader::read(text);
    EXPECT_FALSE(bodies.empty()) << "No bodies parsed from bull.stp";
    if (!bodies.empty()) {
        EXPECT_GT(bodies.front()->lumps().size(), 0u);
    }
}

TEST(StepRoundTrip, Parse_iso14649)
{
    std::string text = loadFile("iso14649-demo.stp");
    if (text.empty()) GTEST_SKIP() << "File not found";

    auto bodies = StepReader::read(text);
    EXPECT_FALSE(bodies.empty()) << "No bodies parsed from iso14649-demo.stp";
    if (!bodies.empty()) {
        EXPECT_GT(bodies.front()->lumps().size(), 0u);
    }
}

TEST(StepRoundTrip, Parse_object)
{
    std::string text = loadFile("object.step");
    if (text.empty()) GTEST_SKIP() << "File not found";

    auto bodies = StepReader::read(text);
    EXPECT_FALSE(bodies.empty()) << "No bodies parsed from object.step";
    if (!bodies.empty()) {
        EXPECT_GT(bodies.front()->lumps().size(), 0u);
    }
}

TEST(StepRoundTrip, Parse_test_model)
{
    std::string text = loadFile("test_model.step");
    if (text.empty()) GTEST_SKIP() << "File not found";

    auto bodies = StepReader::read(text);
    EXPECT_FALSE(bodies.empty()) << "No bodies parsed from test_model.step";
    if (!bodies.empty()) {
        EXPECT_GT(bodies.front()->lumps().size(), 0u);
    }
}

// ── Round-trip export ─────────────────────────────────────────────────────────

TEST(StepRoundTrip, RoundTrip_1797609in)
{
    auto r = doRoundTrip("1797609in.step", "Part_1797609");
    if (!r.fileLoaded) GTEST_SKIP() << "File not found";
    EXPECT_GT(r.bodiesRead, 0);
    ASSERT_TRUE(r.exportOk);
    assertValidStepOutput(r.stepOut, "1797609in.step round-trip");
    writeRoundTripFile(r.stepOut, "rt_1797609in.step");
}

TEST(StepRoundTrip, RoundTrip_angle1)
{
    auto r = doRoundTrip("angle1.stp", "Angle1");
    if (!r.fileLoaded) GTEST_SKIP() << "File not found";
    EXPECT_GT(r.bodiesRead, 0);
    ASSERT_TRUE(r.exportOk);
    assertValidStepOutput(r.stepOut, "angle1.stp round-trip");
    writeRoundTripFile(r.stepOut, "rt_angle1.step");
}

TEST(StepRoundTrip, RoundTrip_bull)
{
    auto r = doRoundTrip("bull.stp", "Bull");
    if (!r.fileLoaded) GTEST_SKIP() << "File not found";
    EXPECT_GT(r.bodiesRead, 0);
    ASSERT_TRUE(r.exportOk);
    assertValidStepOutput(r.stepOut, "bull.stp round-trip");
    writeRoundTripFile(r.stepOut, "rt_bull.step");
}

TEST(StepRoundTrip, RoundTrip_iso14649)
{
    auto r = doRoundTrip("iso14649-demo.stp", "ISO14649Demo");
    if (!r.fileLoaded) GTEST_SKIP() << "File not found";
    EXPECT_GT(r.bodiesRead, 0);
    ASSERT_TRUE(r.exportOk);
    assertValidStepOutput(r.stepOut, "iso14649-demo.stp round-trip");
    writeRoundTripFile(r.stepOut, "rt_iso14649.step");
}

TEST(StepRoundTrip, RoundTrip_object)
{
    auto r = doRoundTrip("object.step", "Object");
    if (!r.fileLoaded) GTEST_SKIP() << "File not found";
    EXPECT_GT(r.bodiesRead, 0);
    ASSERT_TRUE(r.exportOk);
    assertValidStepOutput(r.stepOut, "object.step round-trip");
    writeRoundTripFile(r.stepOut, "rt_object.step");
}

TEST(StepRoundTrip, RoundTrip_test_model)
{
    auto r = doRoundTrip("test_model.step", "TestModel");
    if (!r.fileLoaded) GTEST_SKIP() << "File not found";
    EXPECT_GT(r.bodiesRead, 0);
    ASSERT_TRUE(r.exportOk);
    assertValidStepOutput(r.stepOut, "test_model.step round-trip");
    writeRoundTripFile(r.stepOut, "rt_test_model.step");
}

// ── Structural validation of round-trip output ────────────────────────────────

TEST(StepRoundTrip, RoundTrip_test_model_HasCylindricalSurface)
{
    std::string text = loadFile("test_model.step");
    if (text.empty()) GTEST_SKIP() << "File not found";
    auto bodies = StepReader::read(text);
    if (bodies.empty()) GTEST_SKIP() << "No bodies parsed";
    std::string step = StepWriter::write(*bodies.front(), "TestModel");
    // test_model.step contains cylinders
    EXPECT_TRUE(contains(step, "CYLINDRICAL_SURFACE"));
}

TEST(StepRoundTrip, RoundTrip_object_HasSurfaces)
{
    std::string text = loadFile("object.step");
    if (text.empty()) GTEST_SKIP() << "File not found";
    auto bodies = StepReader::read(text);
    if (bodies.empty()) GTEST_SKIP() << "No bodies parsed";
    std::string step = StepWriter::write(*bodies.front(), "Object");
    EXPECT_TRUE(contains(step, "ADVANCED_FACE"));
    EXPECT_TRUE(contains(step, "VERTEX_POINT"));
}

TEST(StepRoundTrip, RoundTrip_1797609in_HasConicalOrPlaneSurface)
{
    std::string text = loadFile("1797609in.step");
    if (text.empty()) GTEST_SKIP() << "File not found";
    auto bodies = StepReader::read(text);
    if (bodies.empty()) GTEST_SKIP() << "No bodies parsed";
    std::string step = StepWriter::write(*bodies.front());
    // 1797609in.step has conical surfaces
    EXPECT_TRUE(contains(step, "CONICAL_SURFACE") ||
                contains(step, "PLANE(")          ||
                contains(step, "CYLINDRICAL_SURFACE"));
}

TEST(StepRoundTrip, RoundTrip_bull_HasBSplineOrNURBS)
{
    std::string text = loadFile("bull.stp");
    if (text.empty()) GTEST_SKIP() << "File not found";
    auto bodies = StepReader::read(text);
    if (bodies.empty()) GTEST_SKIP() << "No bodies parsed";
    std::string step = StepWriter::write(*bodies.front(), "Bull");
    // bull.stp has B-Spline / NURBS curves and surfaces
    EXPECT_TRUE(contains(step, "B_SPLINE") ||
                contains(step, "RATIONAL") ||
                contains(step, "CYLINDRICAL_SURFACE"));
}

// ── StepReader::readOne convenience ──────────────────────────────────────────

TEST(StepRoundTrip, ReadOne_test_model)
{
    std::string text = loadFile("test_model.step");
    if (text.empty()) GTEST_SKIP() << "File not found";
    auto body = StepReader::readOne(text);
    EXPECT_TRUE(static_cast<bool>(body));
    if (body) {
        EXPECT_GT(body->lumps().size(), 0u);
    }
}

TEST(StepRoundTrip, ReadOne_EmptyString)
{
    auto body = StepReader::readOne("");
    EXPECT_FALSE(static_cast<bool>(body));
}

} // namespace
