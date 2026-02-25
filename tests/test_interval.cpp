#include "GkTest.h"
#include "gk/math/Interval.h"
#include <string>

using gk::Interval;

// ── Construction ────────────────────────────────────────────────────────────
GK_TEST(Interval, DefaultIsZeroPoint)
{
    constexpr Interval i;
    GK_ASSERT_EQ(i.lo, 0.0);
    GK_ASSERT_EQ(i.hi, 0.0);
}

GK_TEST(Interval, ValueConstruction)
{
    constexpr Interval i{1.0, 3.0};
    GK_ASSERT_EQ(i.lo, 1.0);
    GK_ASSERT_EQ(i.hi, 3.0);
}

GK_TEST(Interval, MakeSwaps)
{
    constexpr Interval i = Interval::make(5.0, 2.0);
    GK_ASSERT_EQ(i.lo, 2.0);
    GK_ASSERT_EQ(i.hi, 5.0);
}

GK_TEST(Interval, PointFactory)
{
    constexpr Interval i = Interval::point(3.14);
    GK_ASSERT_TRUE(i.isPoint());
    GK_ASSERT_NEAR(i.lo, 3.14, 1e-12);
    GK_ASSERT_NEAR(i.hi, 3.14, 1e-12);
}

// ── Properties ──────────────────────────────────────────────────────────────
GK_TEST(Interval, Width)
{
    constexpr Interval i{1.0, 5.0};
    GK_ASSERT_NEAR(i.width(), 4.0, 1e-12);
}

GK_TEST(Interval, Midpoint)
{
    constexpr Interval i{0.0, 10.0};
    GK_ASSERT_NEAR(i.midpoint(), 5.0, 1e-12);
}

GK_TEST(Interval, IsEmpty)
{
    // lo > hi is empty (invalid but constructible)
    Interval i{3.0, 1.0}; // deliberately inverted, not via make()
    GK_ASSERT_TRUE(i.isEmpty());
    Interval ok{1.0, 3.0};
    GK_ASSERT_FALSE(ok.isEmpty());
}

GK_TEST(Interval, Contains)
{
    constexpr Interval i{0.0, 10.0};
    GK_ASSERT_TRUE(i.contains(5.0));
    GK_ASSERT_TRUE(i.contains(0.0));
    GK_ASSERT_TRUE(i.contains(10.0));
    GK_ASSERT_FALSE(i.contains(-1.0));
    GK_ASSERT_FALSE(i.contains(11.0));
}

GK_TEST(Interval, ContainsInterval)
{
    constexpr Interval outer{0.0, 10.0};
    constexpr Interval inner{2.0, 8.0};
    GK_ASSERT_TRUE(outer.contains(inner));
    GK_ASSERT_FALSE(inner.contains(outer));
}

GK_TEST(Interval, Overlaps)
{
    constexpr Interval a{0.0, 5.0};
    constexpr Interval b{3.0, 8.0};
    constexpr Interval c{6.0, 9.0};
    GK_ASSERT_TRUE(a.overlaps(b));
    GK_ASSERT_FALSE(a.overlaps(c));
}

// ── Arithmetic ──────────────────────────────────────────────────────────────
GK_TEST(Interval, Addition)
{
    constexpr Interval a{1.0, 3.0};
    constexpr Interval b{2.0, 4.0};
    constexpr Interval r = a + b;
    GK_ASSERT_NEAR(r.lo, 3.0, 1e-12);
    GK_ASSERT_NEAR(r.hi, 7.0, 1e-12);
}

GK_TEST(Interval, Subtraction)
{
    constexpr Interval a{5.0, 8.0};
    constexpr Interval b{1.0, 3.0};
    constexpr Interval r = a - b;
    GK_ASSERT_NEAR(r.lo, 2.0, 1e-12);
    GK_ASSERT_NEAR(r.hi, 7.0, 1e-12);
}

GK_TEST(Interval, ScalarMultiplicationPositive)
{
    constexpr Interval i{1.0, 3.0};
    constexpr Interval r = i * 2.0;
    GK_ASSERT_NEAR(r.lo, 2.0, 1e-12);
    GK_ASSERT_NEAR(r.hi, 6.0, 1e-12);
}

GK_TEST(Interval, ScalarMultiplicationNegative)
{
    constexpr Interval i{1.0, 3.0};
    constexpr Interval r = i * (-1.0);
    GK_ASSERT_NEAR(r.lo, -3.0, 1e-12);
    GK_ASSERT_NEAR(r.hi, -1.0, 1e-12);
}

GK_TEST(Interval, Hull)
{
    constexpr Interval a{1.0, 4.0};
    constexpr Interval b{3.0, 7.0};
    constexpr Interval h = a.hull(b);
    GK_ASSERT_NEAR(h.lo, 1.0, 1e-12);
    GK_ASSERT_NEAR(h.hi, 7.0, 1e-12);
}

GK_TEST(Interval, Intersection)
{
    constexpr Interval a{0.0, 6.0};
    constexpr Interval b{3.0, 9.0};
    constexpr Interval r = a.intersection(b);
    GK_ASSERT_NEAR(r.lo, 3.0, 1e-12);
    GK_ASSERT_NEAR(r.hi, 6.0, 1e-12);
}

GK_TEST(Interval, EmptyIntersection)
{
    constexpr Interval a{0.0, 2.0};
    constexpr Interval b{5.0, 8.0};
    constexpr Interval r = a.intersection(b);
    GK_ASSERT_TRUE(r.isEmpty());
}

// ── Comparison ──────────────────────────────────────────────────────────────
GK_TEST(Interval, FuzzyEquals)
{
    Interval a{1.0, 3.0};
    Interval b{1.0 + 1e-11, 3.0 - 1e-11};
    GK_ASSERT_TRUE(a.fuzzyEquals(b));
    GK_ASSERT_FALSE(a.fuzzyEquals(Interval{1.0 + 1e-9, 3.0}));
}

GK_TEST(Interval, ExactEquality)
{
    constexpr Interval a{2.0, 5.0};
    GK_ASSERT_EQ(a, Interval(2.0, 5.0));
    GK_ASSERT_NE(a, Interval(2.0, 6.0));
}

// ── Serialization ───────────────────────────────────────────────────────────
GK_TEST(Interval, ToString)
{
    std::string s = Interval{1.0, 2.0}.toString();
    GK_ASSERT_TRUE(s.find("\"lo\"") != std::string::npos);
    GK_ASSERT_TRUE(s.find("\"hi\"") != std::string::npos);
}

int main()
{
    return gk::test::runAll();
}
