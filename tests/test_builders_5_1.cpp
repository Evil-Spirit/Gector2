// Chapter 5.1 — Box, Sphere, Cylinder, Cone builders

#include "gk/builders/PrimitiveBuilders.h"
#include "gk/brep/BRepQuery.h"
#include "gk/brep/StepExport.h"
#include "gk/surface/SurfaceUtils.h"
#include <filesystem>
#include <gtest/gtest.h>
#include <cmath>

using namespace gk;

// Returns "step_test_output/<filename>" and ensures the directory exists.
static std::string builderStepPath(const std::string& filename) {
    std::filesystem::create_directories("step_test_output");
    return "step_test_output/" + filename;
}

TEST(Builders5_1, BoxVolume) {
    auto body = makeBox(Vec3::zero(), Vec3{1,1,1});
    auto mp = BRepQuery::computeMassProperties(*body, 20);
    EXPECT_NEAR(mp.volume, 1.0, 0.01);
    EXPECT_NEAR(mp.surfaceArea, 6.0, 0.05);
}

TEST(Builders5_1, BoxShifted) {
    auto body = makeBox(Vec3{1,2,3}, Vec3{4,5,7});
    auto mp = BRepQuery::computeMassProperties(*body, 20);
    EXPECT_NEAR(mp.volume, 3.0*3.0*4.0, 0.5);
}

TEST(Builders5_1, SphereVolume) {
    auto body = makeSphere(Vec3::zero(), 1.0);
    auto mp = BRepQuery::computeMassProperties(*body, 32);
    constexpr double kPi = 3.14159265358979323846;
    EXPECT_NEAR(mp.volume, 4.0/3.0*kPi, 0.05);
}

TEST(Builders5_1, CylinderVolume) {
    auto body = makeCylinder(Vec3::zero(), Vec3::unitZ(), 1.0, 1.0);
    auto mp = BRepQuery::computeMassProperties(*body, 32);
    constexpr double kPi = 3.14159265358979323846;
    EXPECT_NEAR(mp.volume, kPi, 0.05);
}

TEST(Builders5_1, ConeVolume) {
    constexpr double kPi = 3.14159265358979323846;
    double halfAngle = kPi / 6.0; // 30 degrees
    double h = 1.0;
    double r = h * std::tan(halfAngle);
    auto body = makeCone(Vec3::zero(), Vec3::unitZ(), halfAngle, h);
    auto mp = BRepQuery::computeMassProperties(*body, 32);
    double expectedVol = kPi * r * r * h / 3.0;
    EXPECT_NEAR(mp.volume, expectedVol, 0.05);
}

TEST(Builders5_1, BoxStepExport) {
    auto body = makeBox(Vec3::zero(), Vec3{2,1,1.5});
    EXPECT_TRUE(StepExport::writeBodyToFile(*body, builderStepPath("box_debug.stp")));
}

TEST(Builders5_1, SphereStepExport) {
    auto body = makeSphere(Vec3::zero(), 1.0);
    EXPECT_TRUE(StepExport::writeBodyToFile(*body, builderStepPath("sphere_builder_debug.stp")));
}

TEST(Builders5_1, CylinderStepExport) {
    auto body = makeCylinder(Vec3::zero(), Vec3::unitZ(), 1.0, 2.0);
    EXPECT_TRUE(StepExport::writeBodyToFile(*body, builderStepPath("cylinder_builder_debug.stp")));
}

TEST(Builders5_1, ConeStepExport) {
    constexpr double kPi = 3.14159265358979323846;
    auto body = makeCone(Vec3::zero(), Vec3::unitZ(), kPi/6.0, 2.0);
    EXPECT_TRUE(StepExport::writeBodyToFile(*body, builderStepPath("cone_builder_debug.stp")));
}
