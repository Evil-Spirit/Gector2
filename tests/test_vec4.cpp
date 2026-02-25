#include "GkTest.h"
#include "gk/math/Vec4.h"
#include <string>

using gk::Vec4;

GK_TEST(Vec4, DefaultConstruction)
{
    constexpr Vec4 v;
    GK_ASSERT_EQ(v.x, 0.0);
    GK_ASSERT_EQ(v.y, 0.0);
    GK_ASSERT_EQ(v.z, 0.0);
    GK_ASSERT_EQ(v.w, 0.0);
}

GK_TEST(Vec4, ValueConstruction)
{
    constexpr Vec4 v{1,2,3,4};
    GK_ASSERT_EQ(v.x, 1.0);
    GK_ASSERT_EQ(v.y, 2.0);
    GK_ASSERT_EQ(v.z, 3.0);
    GK_ASSERT_EQ(v.w, 4.0);
}

GK_TEST(Vec4, StaticFactories)
{
    GK_ASSERT_EQ(Vec4::zero(),  Vec4(0,0,0,0));
    GK_ASSERT_EQ(Vec4::unitX(), Vec4(1,0,0,0));
    GK_ASSERT_EQ(Vec4::unitY(), Vec4(0,1,0,0));
    GK_ASSERT_EQ(Vec4::unitZ(), Vec4(0,0,1,0));
    GK_ASSERT_EQ(Vec4::unitW(), Vec4(0,0,0,1));
}

GK_TEST(Vec4, Arithmetic)
{
    constexpr Vec4 a{1,2,3,4};
    constexpr Vec4 b{5,6,7,8};
    GK_ASSERT_EQ(a + b, Vec4(6,8,10,12));
    GK_ASSERT_EQ(b - a, Vec4(4,4,4,4));
    GK_ASSERT_EQ(a * 2.0, Vec4(2,4,6,8));
    GK_ASSERT_EQ(2.0 * a, Vec4(2,4,6,8));
    GK_ASSERT_EQ(b / 2.0, Vec4(2.5,3,3.5,4));
    GK_ASSERT_EQ(-a, Vec4(-1,-2,-3,-4));
}

GK_TEST(Vec4, Dot)
{
    constexpr Vec4 a{1,0,0,0};
    constexpr Vec4 b{0,1,0,0};
    GK_ASSERT_NEAR(a.dot(b), 0.0, 1e-12);
    constexpr Vec4 c{1,2,3,4};
    GK_ASSERT_NEAR(c.dot(c), 30.0, 1e-12);
}

GK_TEST(Vec4, Norm)
{
    constexpr Vec4 ux = Vec4::unitX();
    GK_ASSERT_NEAR(ux.norm(), 1.0, 1e-12);
    constexpr Vec4 uw = Vec4{0,0,0,2};
    GK_ASSERT_NEAR(uw.norm(), 2.0, 1e-12);
}

GK_TEST(Vec4, Normalized)
{
    Vec4 n = Vec4{1,1,1,1}.normalized();
    GK_ASSERT_NEAR(n.norm(), 1.0, 1e-12);
}

GK_TEST(Vec4, FuzzyEquals)
{
    Vec4 a{1,2,3,4};
    Vec4 b{1+1e-11, 2, 3, 4};
    GK_ASSERT_TRUE(a.fuzzyEquals(b));
    GK_ASSERT_FALSE(a.fuzzyEquals(Vec4{1+1e-9,2,3,4}));
}

GK_TEST(Vec4, ToString)
{
    std::string s = Vec4{1,2,3,4}.toString();
    GK_ASSERT_TRUE(s.find("\"x\"") != std::string::npos);
    GK_ASSERT_TRUE(s.find("\"w\"") != std::string::npos);
}

int main()
{
    return gk::test::runAll();
}
