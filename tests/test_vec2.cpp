#include "GkTest.h"
#include "gk/math/Vec2.h"
#include <cmath>
#include <string>

using gk::Vec2;

// ── Construction ────────────────────────────────────────────────────────────
GK_TEST(Vec2, DefaultConstruction)
{
    constexpr Vec2 v;
    GK_ASSERT_EQ(v.x, 0.0);
    GK_ASSERT_EQ(v.y, 0.0);
}

GK_TEST(Vec2, ValueConstruction)
{
    constexpr Vec2 v{3.0, 4.0};
    GK_ASSERT_EQ(v.x, 3.0);
    GK_ASSERT_EQ(v.y, 4.0);
}

GK_TEST(Vec2, StaticFactories)
{
    GK_ASSERT_EQ(Vec2::zero(),  Vec2(0.0, 0.0));
    GK_ASSERT_EQ(Vec2::unitX(), Vec2(1.0, 0.0));
    GK_ASSERT_EQ(Vec2::unitY(), Vec2(0.0, 1.0));
}

// ── Arithmetic ──────────────────────────────────────────────────────────────
GK_TEST(Vec2, Addition)
{
    constexpr Vec2 a{1.0, 2.0};
    constexpr Vec2 b{3.0, 4.0};
    constexpr Vec2 c = a + b;
    GK_ASSERT_EQ(c.x, 4.0);
    GK_ASSERT_EQ(c.y, 6.0);
}

GK_TEST(Vec2, Subtraction)
{
    constexpr Vec2 a{5.0, 3.0};
    constexpr Vec2 b{2.0, 1.0};
    constexpr Vec2 c = a - b;
    GK_ASSERT_EQ(c.x, 3.0);
    GK_ASSERT_EQ(c.y, 2.0);
}

GK_TEST(Vec2, ScalarMultiplication)
{
    constexpr Vec2 v{2.0, 3.0};
    constexpr Vec2 r = v * 2.0;
    GK_ASSERT_EQ(r.x, 4.0);
    GK_ASSERT_EQ(r.y, 6.0);
    constexpr Vec2 r2 = 2.0 * v;
    GK_ASSERT_EQ(r2, r);
}

GK_TEST(Vec2, ScalarDivision)
{
    constexpr Vec2 v{4.0, 6.0};
    constexpr Vec2 r = v / 2.0;
    GK_ASSERT_EQ(r.x, 2.0);
    GK_ASSERT_EQ(r.y, 3.0);
}

GK_TEST(Vec2, Negation)
{
    constexpr Vec2 v{1.0, -2.0};
    constexpr Vec2 n = -v;
    GK_ASSERT_EQ(n.x, -1.0);
    GK_ASSERT_EQ(n.y,  2.0);
}

GK_TEST(Vec2, CompoundAssignment)
{
    Vec2 v{1.0, 2.0};
    v += Vec2{3.0, 4.0};
    GK_ASSERT_EQ(v, Vec2(4.0, 6.0));
    v -= Vec2{1.0, 1.0};
    GK_ASSERT_EQ(v, Vec2(3.0, 5.0));
    v *= 2.0;
    GK_ASSERT_EQ(v, Vec2(6.0, 10.0));
    v /= 2.0;
    GK_ASSERT_EQ(v, Vec2(3.0, 5.0));
}

// ── Geometry ────────────────────────────────────────────────────────────────
GK_TEST(Vec2, Dot)
{
    constexpr Vec2 a{1.0, 0.0};
    constexpr Vec2 b{0.0, 1.0};
    GK_ASSERT_EQ(a.dot(b), 0.0);
    constexpr Vec2 c{3.0, 4.0};
    GK_ASSERT_NEAR(c.dot(c), 25.0, 1e-12);
}

GK_TEST(Vec2, Cross)
{
    constexpr Vec2 x = Vec2::unitX();
    constexpr Vec2 y = Vec2::unitY();
    GK_ASSERT_NEAR(x.cross(y),  1.0, 1e-12);
    GK_ASSERT_NEAR(y.cross(x), -1.0, 1e-12);
}

GK_TEST(Vec2, Norm)
{
    Vec2 v{3.0, 4.0};
    GK_ASSERT_NEAR(v.norm(), 5.0, 1e-12);
    GK_ASSERT_NEAR(v.squaredNorm(), 25.0, 1e-12);
}

GK_TEST(Vec2, Normalized)
{
    Vec2 v{3.0, 4.0};
    Vec2 n = v.normalized();
    GK_ASSERT_NEAR(n.norm(), 1.0, 1e-12);
    GK_ASSERT_NEAR(n.x, 0.6, 1e-12);
    GK_ASSERT_NEAR(n.y, 0.8, 1e-12);
}

GK_TEST(Vec2, NormalizedZeroVector)
{
    Vec2 v = Vec2::zero();
    Vec2 n = v.normalized();
    GK_ASSERT_EQ(n, Vec2::zero());
}

// ── Comparison ──────────────────────────────────────────────────────────────
GK_TEST(Vec2, FuzzyEquals)
{
    Vec2 a{1.0, 2.0};
    Vec2 b{1.0 + 1e-11, 2.0 - 1e-11};
    GK_ASSERT_TRUE(a.fuzzyEquals(b));
    Vec2 c{1.0 + 1e-9, 2.0};
    GK_ASSERT_FALSE(a.fuzzyEquals(c));
}

GK_TEST(Vec2, ExactEquality)
{
    constexpr Vec2 a{1.0, 2.0};
    constexpr Vec2 b{1.0, 2.0};
    GK_ASSERT_EQ(a, b);
    GK_ASSERT_NE(a, Vec2(1.0, 3.0));
}

// ── Serialization ───────────────────────────────────────────────────────────
GK_TEST(Vec2, ToString)
{
    Vec2 v{0.0, 0.0};
    std::string s = v.toString();
    // Must contain the JSON keys
    GK_ASSERT_TRUE(s.find("\"x\"") != std::string::npos);
    GK_ASSERT_TRUE(s.find("\"y\"") != std::string::npos);
}
