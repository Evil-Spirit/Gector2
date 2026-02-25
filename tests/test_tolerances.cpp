#include "GkTest.h"
#include "gk/Tolerances.h"
#include <cmath>
#include <limits>

// ── ToleranceSet defaults ────────────────────────────────────────────────────
GK_TEST(ToleranceSet, Defaults)
{
    gk::ToleranceSet ts;
    GK_ASSERT_NEAR(ts.linear,     1e-7,  1e-20);
    GK_ASSERT_NEAR(ts.angular,    1e-9,  1e-20);
    GK_ASSERT_NEAR(ts.parametric, 1e-10, 1e-20);
}

// ── Tolerances singleton ─────────────────────────────────────────────────────
GK_TEST(Tolerances, CurrentReturnsDefaults)
{
    const gk::ToleranceSet& t = gk::Tolerances::current();
    GK_ASSERT_NEAR(t.linear,     1e-7,  1e-20);
    GK_ASSERT_NEAR(t.angular,    1e-9,  1e-20);
    GK_ASSERT_NEAR(t.parametric, 1e-10, 1e-20);
}

GK_TEST(Tolerances, ConvenienceAccessors)
{
    GK_ASSERT_NEAR(gk::Tolerances::linear(),     1e-7,  1e-20);
    GK_ASSERT_NEAR(gk::Tolerances::angular(),    1e-9,  1e-20);
    GK_ASSERT_NEAR(gk::Tolerances::parametric(), 1e-10, 1e-20);
}

GK_TEST(Tolerances, PushPop)
{
    gk::ToleranceSet tight;
    tight.linear = 1e-12;

    gk::Tolerances::push(tight);
    GK_ASSERT_NEAR(gk::Tolerances::linear(), 1e-12, 1e-20);

    gk::Tolerances::pop();
    GK_ASSERT_NEAR(gk::Tolerances::linear(), 1e-7, 1e-20);
}

GK_TEST(Tolerances, PopBelowOneEntryIsHarmless)
{
    // Popping with only the default entry should not crash.
    gk::Tolerances::pop();
    // The default entry must still be accessible.
    GK_ASSERT_NEAR(gk::Tolerances::linear(), 1e-7, 1e-20);
}

// ── ToleranceGuard ───────────────────────────────────────────────────────────
GK_TEST(ToleranceGuard, RestoresOnDestruction)
{
    double before = gk::Tolerances::linear();
    {
        gk::ToleranceSet t;
        t.linear = 1e-3;
        gk::ToleranceGuard guard(t);
        GK_ASSERT_NEAR(gk::Tolerances::linear(), 1e-3, 1e-10);
    }
    GK_ASSERT_NEAR(gk::Tolerances::linear(), before, 1e-20);
}

GK_TEST(ToleranceGuard, NestedGuards)
{
    gk::ToleranceSet outer, inner;
    outer.linear = 1e-4;
    inner.linear = 1e-8;

    gk::ToleranceGuard g1(outer);
    GK_ASSERT_NEAR(gk::Tolerances::linear(), 1e-4, 1e-10);
    {
        gk::ToleranceGuard g2(inner);
        GK_ASSERT_NEAR(gk::Tolerances::linear(), 1e-8, 1e-10);
    }
    GK_ASSERT_NEAR(gk::Tolerances::linear(), 1e-4, 1e-10);
}

// ── FuzzyCompare::absolute ───────────────────────────────────────────────────
GK_TEST(FuzzyCompareAbsolute, Equal)
{
    GK_ASSERT_TRUE(gk::FuzzyCompare::absolute(1.0, 1.0, 1e-10));
}

GK_TEST(FuzzyCompareAbsolute, WithinTolerance)
{
    GK_ASSERT_TRUE(gk::FuzzyCompare::absolute(1.0, 1.0 + 1e-11, 1e-10));
}

GK_TEST(FuzzyCompareAbsolute, OutsideTolerance)
{
    GK_ASSERT_FALSE(gk::FuzzyCompare::absolute(1.0, 1.0 + 1e-9, 1e-10));
}

GK_TEST(FuzzyCompareAbsolute, NearZero)
{
    GK_ASSERT_TRUE(gk::FuzzyCompare::absolute(0.0, 1e-11, 1e-10));
    GK_ASSERT_FALSE(gk::FuzzyCompare::absolute(0.0, 1e-9, 1e-10));
}

GK_TEST(FuzzyCompareAbsolute, Negative)
{
    GK_ASSERT_TRUE(gk::FuzzyCompare::absolute(-1.0, -1.0 + 1e-11, 1e-10));
}

// ── FuzzyCompare::relative ───────────────────────────────────────────────────
GK_TEST(FuzzyCompareRelative, Equal)
{
    GK_ASSERT_TRUE(gk::FuzzyCompare::relative(1e10, 1e10, 1e-10));
}

GK_TEST(FuzzyCompareRelative, LargeNumbers)
{
    // |1e10 - (1e10 + 1e3)| / 1e10 = 1e-7 > 1e-10 — outside
    GK_ASSERT_FALSE(gk::FuzzyCompare::relative(1e10, 1e10 + 1e3, 1e-10));
    // |1e10 - (1e10 + 1)| / 1e10 = 1e-10 — right on boundary
    GK_ASSERT_TRUE(gk::FuzzyCompare::relative(1e10, 1e10 + 1.0, 1e-10));
}

GK_TEST(FuzzyCompareRelative, NearZeroUsesScaleOf1)
{
    // scale = max(0,0,1) = 1  → behaves like absolute near zero
    GK_ASSERT_TRUE(gk::FuzzyCompare::relative(0.0, 1e-11, 1e-10));
}

// ── FuzzyCompare::ulp ────────────────────────────────────────────────────────
GK_TEST(FuzzyCompareUlp, ExactEquality)
{
    GK_ASSERT_TRUE(gk::FuzzyCompare::ulp(1.0, 1.0));
}

GK_TEST(FuzzyCompareUlp, AdjacentDoubles)
{
    // nextafter(1.0, 2.0) is 1 ULP away
    double next = std::nextafter(1.0, 2.0);
    GK_ASSERT_TRUE(gk::FuzzyCompare::ulp(1.0, next, 1));
}

GK_TEST(FuzzyCompareUlp, TwoUlpsApart)
{
    double a = 1.0;
    double b = std::nextafter(std::nextafter(a, 2.0), 2.0); // 2 ULPs away
    GK_ASSERT_TRUE(gk::FuzzyCompare::ulp(a, b, 2));
    GK_ASSERT_FALSE(gk::FuzzyCompare::ulp(a, b, 1));
}

GK_TEST(FuzzyCompareUlp, NanIsFalse)
{
    double nan = std::numeric_limits<double>::quiet_NaN();
    GK_ASSERT_FALSE(gk::FuzzyCompare::ulp(nan, nan));
    GK_ASSERT_FALSE(gk::FuzzyCompare::ulp(1.0, nan));
}

GK_TEST(FuzzyCompareUlp, InfEquality)
{
    double inf = std::numeric_limits<double>::infinity();
    GK_ASSERT_TRUE(gk::FuzzyCompare::ulp(inf, inf));
    GK_ASSERT_TRUE(gk::FuzzyCompare::ulp(-inf, -inf));
    GK_ASSERT_FALSE(gk::FuzzyCompare::ulp(inf, -inf));
    GK_ASSERT_FALSE(gk::FuzzyCompare::ulp(inf, 1.0));
}

GK_TEST(FuzzyCompareUlp, PositiveZeroNegativeZero)
{
    GK_ASSERT_TRUE(gk::FuzzyCompare::ulp(0.0, -0.0));
}

GK_TEST(FuzzyCompareUlp, NegativeNumbers)
{
    double a = -1.0;
    double b = std::nextafter(-1.0, -2.0); // 1 ULP away in negative direction
    GK_ASSERT_TRUE(gk::FuzzyCompare::ulp(a, b, 1));
    GK_ASSERT_FALSE(gk::FuzzyCompare::ulp(a, b, 0));
}
