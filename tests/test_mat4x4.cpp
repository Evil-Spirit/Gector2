#include "GkTest.h"
#include "gk/math/Mat4x4.h"
#include <cmath>

using gk::Mat4x4;
using gk::Vec4;

GK_TEST(Mat4x4, DefaultIsZero)
{
    Mat4x4 m;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            GK_ASSERT_EQ(m(r,c), 0.0);
}

GK_TEST(Mat4x4, Identity)
{
    constexpr Mat4x4 I = Mat4x4::identity();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            GK_ASSERT_EQ(I(r,c), (r == c) ? 1.0 : 0.0);
}

GK_TEST(Mat4x4, MatVecMultiplication)
{
    constexpr Mat4x4 I = Mat4x4::identity();
    constexpr Vec4   v{1,2,3,4};
    constexpr Vec4   r = I * v;
    GK_ASSERT_EQ(r, v);
}

GK_TEST(Mat4x4, MatMatMultiplication)
{
    constexpr Mat4x4 I = Mat4x4::identity();
    Mat4x4 a{
        1,2,3,4,
        5,6,7,8,
        9,10,11,12,
        13,14,15,16
    };
    GK_ASSERT_EQ(I * a, a);
    GK_ASSERT_EQ(a * I, a);
}

GK_TEST(Mat4x4, Transpose)
{
    Mat4x4 a{
        1, 2, 3, 4,
        5, 6, 7, 8,
        9,10,11,12,
       13,14,15,16
    };
    Mat4x4 t = a.transposed();
    GK_ASSERT_EQ(t(0,1),  5.0);
    GK_ASSERT_EQ(t(1,0),  2.0);
    GK_ASSERT_EQ(t(3,0),  4.0);
    GK_ASSERT_EQ(t(0,3), 13.0);
}

GK_TEST(Mat4x4, ScalarMultiplication)
{
    constexpr Mat4x4 I = Mat4x4::identity();
    Mat4x4 r = I * 3.0;
    GK_ASSERT_EQ(r(0,0), 3.0);
    GK_ASSERT_EQ(r(0,1), 0.0);
}

GK_TEST(Mat4x4, AdditionSubtraction)
{
    constexpr Mat4x4 I = Mat4x4::identity();
    Mat4x4 r = I + I;
    GK_ASSERT_NEAR(r(0,0), 2.0, 1e-12);
    Mat4x4 z = I - I;
    GK_ASSERT_TRUE(z.fuzzyEquals(Mat4x4::zero()));
}

GK_TEST(Mat4x4, FuzzyEquals)
{
    Mat4x4 a = Mat4x4::identity();
    Mat4x4 b = Mat4x4::identity();
    b(2,2) += 1e-11;
    GK_ASSERT_TRUE(a.fuzzyEquals(b));
    b(2,2) += 1e-8;
    GK_ASSERT_FALSE(a.fuzzyEquals(b));
}

GK_TEST(Mat4x4, ToString)
{
    std::string s = Mat4x4::identity().toString();
    GK_ASSERT_TRUE(s.size() > 0);
}
