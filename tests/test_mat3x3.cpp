#include "GkTest.h"
#include "gk/math/Mat3x3.h"
#include <cmath>

using gk::Mat3x3;
using gk::Vec3;
static constexpr double kPi = 3.14159265358979323846;

GK_TEST(Mat3x3, DefaultIsZero)
{
    Mat3x3 m;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            GK_ASSERT_EQ(m(r,c), 0.0);
}

GK_TEST(Mat3x3, Identity)
{
    Mat3x3 I = Mat3x3::identity();
    GK_ASSERT_EQ(I(0,0), 1.0);
    GK_ASSERT_EQ(I(1,1), 1.0);
    GK_ASSERT_EQ(I(2,2), 1.0);
    GK_ASSERT_EQ(I(0,1), 0.0);
}

GK_TEST(Mat3x3, MatVecMultiplication)
{
    Mat3x3 I = Mat3x3::identity();
    Vec3   v{1,2,3};
    Vec3   r = I * v;
    GK_ASSERT_EQ(r, v);
}

GK_TEST(Mat3x3, MatMatMultiplication)
{
    Mat3x3 I = Mat3x3::identity();
    Mat3x3 a{1,2,3, 4,5,6, 7,8,9};
    GK_ASSERT_EQ(I * a, a);
    GK_ASSERT_EQ(a * I, a);
}

GK_TEST(Mat3x3, Transpose)
{
    Mat3x3 a{1,2,3, 4,5,6, 7,8,9};
    Mat3x3 t = a.transposed();
    GK_ASSERT_EQ(t(0,1), 4.0);
    GK_ASSERT_EQ(t(1,0), 2.0);
    GK_ASSERT_EQ(t(2,0), 3.0);
}

GK_TEST(Mat3x3, Determinant)
{
    Mat3x3 I = Mat3x3::identity();
    GK_ASSERT_NEAR(I.determinant(), 1.0, 1e-12);
    Mat3x3 z = Mat3x3::zero();
    GK_ASSERT_NEAR(z.determinant(), 0.0, 1e-12);
}

GK_TEST(Mat3x3, Inverse)
{
    Mat3x3 a{1,2,3, 0,1,4, 5,6,0};
    Mat3x3 inv = a.inverse();
    Mat3x3 r   = a * inv;
    GK_ASSERT_TRUE(r.fuzzyEquals(Mat3x3::identity(), 1e-9));
}

GK_TEST(Mat3x3, SingularInverse)
{
    // Rows are linearly dependent
    Mat3x3 s{1,2,3, 2,4,6, 0,0,1};
    Mat3x3 inv = s.inverse();
    GK_ASSERT_TRUE(inv.fuzzyEquals(Mat3x3::zero()));
}

GK_TEST(Mat3x3, RotationAxisAngle90)
{
    // Rotate 90° about Z: x-unit should map to y-unit.
    Mat3x3 R = Mat3x3::rotationAxisAngle(Vec3::unitZ(), kPi / 2.0);
    Vec3   r = R * Vec3::unitX();
    GK_ASSERT_TRUE(r.fuzzyEquals(Vec3::unitY(), 1e-9));
}

GK_TEST(Mat3x3, RotationPreservesNorm)
{
    Mat3x3 R = Mat3x3::rotationAxisAngle(Vec3{1,1,1}.normalized(), kPi / 3.0);
    Vec3   v{2,3,4};
    Vec3   rv = R * v;
    GK_ASSERT_NEAR(rv.norm(), v.norm(), 1e-9);
}

GK_TEST(Mat3x3, FuzzyEquals)
{
    Mat3x3 a = Mat3x3::identity();
    Mat3x3 b = Mat3x3::identity();
    b(0,0) += 1e-11;
    GK_ASSERT_TRUE(a.fuzzyEquals(b));
    b(0,0) += 1e-8;
    GK_ASSERT_FALSE(a.fuzzyEquals(b));
}

GK_TEST(Mat3x3, ToString)
{
    std::string s = Mat3x3::identity().toString();
    GK_ASSERT_TRUE(s.size() > 0);
    GK_ASSERT_TRUE(s.front() == '[');
}
