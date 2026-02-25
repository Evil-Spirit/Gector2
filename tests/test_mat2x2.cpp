#include "GkTest.h"
#include "gk/math/Mat2x2.h"
#include <cmath>

using gk::Mat2x2;
using gk::Vec2;

GK_TEST(Mat2x2, DefaultIsZero)
{
    Mat2x2 m;
    GK_ASSERT_EQ(m(0,0), 0.0);
    GK_ASSERT_EQ(m(1,1), 0.0);
}

GK_TEST(Mat2x2, Identity)
{
    constexpr Mat2x2 I = Mat2x2::identity();
    GK_ASSERT_EQ(I(0,0), 1.0);
    GK_ASSERT_EQ(I(0,1), 0.0);
    GK_ASSERT_EQ(I(1,0), 0.0);
    GK_ASSERT_EQ(I(1,1), 1.0);
}

GK_TEST(Mat2x2, Addition)
{
    constexpr Mat2x2 a{1,2,3,4};
    constexpr Mat2x2 b{5,6,7,8};
    constexpr Mat2x2 c = a + b;
    GK_ASSERT_EQ(c(0,0), 6.0);
    GK_ASSERT_EQ(c(1,1), 12.0);
}

GK_TEST(Mat2x2, Subtraction)
{
    constexpr Mat2x2 a{5,6,7,8};
    constexpr Mat2x2 b{1,2,3,4};
    constexpr Mat2x2 c = a - b;
    GK_ASSERT_EQ(c(0,0), 4.0);
    GK_ASSERT_EQ(c(1,1), 4.0);
}

GK_TEST(Mat2x2, ScalarMultiplication)
{
    constexpr Mat2x2 a{1,2,3,4};
    constexpr Mat2x2 r = a * 2.0;
    GK_ASSERT_EQ(r(0,0), 2.0);
    GK_ASSERT_EQ(r(0,1), 4.0);
    GK_ASSERT_EQ(2.0 * a, r);
}

GK_TEST(Mat2x2, MatrixMultiplication)
{
    constexpr Mat2x2 I = Mat2x2::identity();
    constexpr Mat2x2 a{1,2,3,4};
    GK_ASSERT_EQ(I * a, a);
    GK_ASSERT_EQ(a * I, a);
}

GK_TEST(Mat2x2, MatVecMultiplication)
{
    constexpr Mat2x2 a{1,0,0,2};
    constexpr Vec2   v{3,4};
    constexpr Vec2   r = a * v;
    GK_ASSERT_EQ(r.x, 3.0);
    GK_ASSERT_EQ(r.y, 8.0);
}

GK_TEST(Mat2x2, Transpose)
{
    constexpr Mat2x2 a{1,2,3,4};
    constexpr Mat2x2 t = a.transposed();
    GK_ASSERT_EQ(t(0,1), 3.0);
    GK_ASSERT_EQ(t(1,0), 2.0);
}

GK_TEST(Mat2x2, Determinant)
{
    constexpr Mat2x2 a{3,8,4,6};
    GK_ASSERT_NEAR(a.determinant(), 3*6 - 8*4, 1e-12);
}

GK_TEST(Mat2x2, Inverse)
{
    Mat2x2 a{4,7,2,6};
    Mat2x2 inv = a.inverse();
    Mat2x2 r   = a * inv;
    GK_ASSERT_TRUE(r.fuzzyEquals(Mat2x2::identity()));
}

GK_TEST(Mat2x2, SingularInverse)
{
    Mat2x2 s{1,2,2,4}; // determinant = 0
    Mat2x2 inv = s.inverse();
    GK_ASSERT_TRUE(inv.fuzzyEquals(Mat2x2::zero()));
}

GK_TEST(Mat2x2, FuzzyEquals)
{
    Mat2x2 a{1,2,3,4};
    Mat2x2 b{1+1e-11, 2, 3, 4};
    GK_ASSERT_TRUE(a.fuzzyEquals(b));
    GK_ASSERT_FALSE(a.fuzzyEquals(Mat2x2{1+1e-9, 2, 3, 4}));
}

GK_TEST(Mat2x2, ToString)
{
    std::string s = Mat2x2::identity().toString();
    GK_ASSERT_TRUE(s.size() > 0);
}
