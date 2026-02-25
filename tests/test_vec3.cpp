#include "GkTest.h"
#include "gk/math/Vec3.h"
#include <cmath>
#include <string>

using gk::Vec3;
static constexpr double kPi = 3.14159265358979323846;

// ── Construction ────────────────────────────────────────────────────────────
GK_TEST(Vec3, DefaultConstruction)
{
    constexpr Vec3 v;
    GK_ASSERT_EQ(v.x, 0.0);
    GK_ASSERT_EQ(v.y, 0.0);
    GK_ASSERT_EQ(v.z, 0.0);
}

GK_TEST(Vec3, ValueConstruction)
{
    constexpr Vec3 v{1.0, 2.0, 3.0};
    GK_ASSERT_EQ(v.x, 1.0);
    GK_ASSERT_EQ(v.y, 2.0);
    GK_ASSERT_EQ(v.z, 3.0);
}

GK_TEST(Vec3, StaticFactories)
{
    GK_ASSERT_EQ(Vec3::zero(),  Vec3(0,0,0));
    GK_ASSERT_EQ(Vec3::unitX(), Vec3(1,0,0));
    GK_ASSERT_EQ(Vec3::unitY(), Vec3(0,1,0));
    GK_ASSERT_EQ(Vec3::unitZ(), Vec3(0,0,1));
}

// ── Arithmetic ──────────────────────────────────────────────────────────────
GK_TEST(Vec3, Addition)
{
    constexpr Vec3 r = Vec3{1,2,3} + Vec3{4,5,6};
    GK_ASSERT_EQ(r, Vec3(5,7,9));
}

GK_TEST(Vec3, Subtraction)
{
    constexpr Vec3 r = Vec3{4,5,6} - Vec3{1,2,3};
    GK_ASSERT_EQ(r, Vec3(3,3,3));
}

GK_TEST(Vec3, ScalarMultiplication)
{
    constexpr Vec3 v{1,2,3};
    constexpr Vec3 r = v * 3.0;
    GK_ASSERT_EQ(r, Vec3(3,6,9));
    constexpr Vec3 r2 = 3.0 * v;
    GK_ASSERT_EQ(r2, r);
}

GK_TEST(Vec3, ScalarDivision)
{
    constexpr Vec3 r = Vec3{3,6,9} / 3.0;
    GK_ASSERT_EQ(r, Vec3(1,2,3));
}

GK_TEST(Vec3, Negation)
{
    constexpr Vec3 r = -Vec3{1,-2,3};
    GK_ASSERT_EQ(r, Vec3(-1,2,-3));
}

GK_TEST(Vec3, CompoundAssignment)
{
    Vec3 v{1,2,3};
    v += Vec3{1,1,1};
    GK_ASSERT_EQ(v, Vec3(2,3,4));
    v -= Vec3{2,3,4};
    GK_ASSERT_EQ(v, Vec3(0,0,0));
    v  = Vec3{2,4,6};
    v *= 0.5;
    GK_ASSERT_EQ(v, Vec3(1,2,3));
    v /= 1.0;
    GK_ASSERT_EQ(v, Vec3(1,2,3));
}

// ── Geometry ────────────────────────────────────────────────────────────────
GK_TEST(Vec3, Dot)
{
    GK_ASSERT_NEAR(Vec3::unitX().dot(Vec3::unitY()), 0.0, 1e-12);
    constexpr Vec3 a{1,2,3};
    constexpr Vec3 b{4,5,6};
    GK_ASSERT_NEAR(a.dot(b), 32.0, 1e-12);
}

GK_TEST(Vec3, Cross)
{
    constexpr Vec3 r = Vec3::unitX().cross(Vec3::unitY());
    GK_ASSERT_TRUE(r.fuzzyEquals(Vec3::unitZ()));
    constexpr Vec3 r2 = Vec3::unitY().cross(Vec3::unitX());
    GK_ASSERT_TRUE(r2.fuzzyEquals(-Vec3::unitZ()));
}

GK_TEST(Vec3, CrossAnticommutative)
{
    Vec3 a{1,2,3};
    Vec3 b{4,5,6};
    GK_ASSERT_TRUE((a.cross(b)).fuzzyEquals(-(b.cross(a))));
}

GK_TEST(Vec3, Norm)
{
    constexpr Vec3 ux = Vec3::unitX();
    GK_ASSERT_NEAR(ux.norm(), 1.0, 1e-12);
    constexpr Vec3 v{3,4,0};
    GK_ASSERT_NEAR(v.norm(), 5.0, 1e-12);
    constexpr Vec3 w{1,1,1};
    GK_ASSERT_NEAR(w.squaredNorm(), 3.0, 1e-12);
}

GK_TEST(Vec3, Normalized)
{
    Vec3 n = Vec3{1,2,2}.normalized();
    GK_ASSERT_NEAR(n.norm(), 1.0, 1e-12);
}

GK_TEST(Vec3, NormalizedZero)
{
    GK_ASSERT_EQ(Vec3::zero().normalized(), Vec3::zero());
}

// ── Comparison ──────────────────────────────────────────────────────────────
GK_TEST(Vec3, FuzzyEquals)
{
    Vec3 a{1,2,3};
    Vec3 b{1+1e-11, 2-1e-11, 3+1e-11};
    GK_ASSERT_TRUE(a.fuzzyEquals(b));
    GK_ASSERT_FALSE(a.fuzzyEquals(Vec3{1+1e-9, 2, 3}));
}

// ── Serialization ───────────────────────────────────────────────────────────
GK_TEST(Vec3, ToString)
{
    std::string s = Vec3{1,2,3}.toString();
    GK_ASSERT_TRUE(s.find("\"x\"") != std::string::npos);
    GK_ASSERT_TRUE(s.find("\"y\"") != std::string::npos);
    GK_ASSERT_TRUE(s.find("\"z\"") != std::string::npos);
}

int main()
{
    return gk::test::runAll();
}
