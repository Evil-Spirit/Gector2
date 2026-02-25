#include "GkTest.h"
#include "gk/math/Quaternion.h"
#include <cmath>

using gk::Quaternion;
using gk::Vec3;
using gk::Mat3x3;
static constexpr double kPi = 3.14159265358979323846;

// ── Construction ────────────────────────────────────────────────────────────
GK_TEST(Quaternion, DefaultIsIdentity)
{
    constexpr Quaternion q;
    GK_ASSERT_EQ(q.x, 0.0);
    GK_ASSERT_EQ(q.y, 0.0);
    GK_ASSERT_EQ(q.z, 0.0);
    GK_ASSERT_EQ(q.w, 1.0);
}

GK_TEST(Quaternion, IdentityFactory)
{
    constexpr Quaternion q = Quaternion::identity();
    GK_ASSERT_EQ(q.w, 1.0);
    GK_ASSERT_EQ(q.x, 0.0);
}

GK_TEST(Quaternion, FromAxisAngle)
{
    Quaternion q = Quaternion::fromAxisAngle(Vec3::unitZ(), kPi);
    GK_ASSERT_NEAR(q.norm(), 1.0, 1e-12);
    // 180° about Z: should flip X and Y
    Vec3 r = q.rotate(Vec3::unitX());
    GK_ASSERT_TRUE(r.fuzzyEquals(-Vec3::unitX(), 1e-9));
}

// ── Arithmetic ──────────────────────────────────────────────────────────────
GK_TEST(Quaternion, IdentityMultiplication)
{
    Quaternion id = Quaternion::identity();
    Quaternion q  = Quaternion::fromAxisAngle(Vec3::unitY(), kPi / 4.0);
    Quaternion r  = id * q;
    GK_ASSERT_TRUE(r.fuzzyEquals(q));
}

GK_TEST(Quaternion, Conjugate)
{
    Quaternion q = Quaternion::fromAxisAngle(Vec3::unitZ(), kPi / 2.0);
    Quaternion c = q.conjugate();
    GK_ASSERT_NEAR(c.w,  q.w, 1e-12);
    GK_ASSERT_NEAR(c.x, -q.x, 1e-12);
    GK_ASSERT_NEAR(c.y, -q.y, 1e-12);
    GK_ASSERT_NEAR(c.z, -q.z, 1e-12);
}

GK_TEST(Quaternion, QuaternionTimesConjugateIsIdentity)
{
    Quaternion q = Quaternion::fromAxisAngle(Vec3{1,1,0}.normalized(), kPi / 3.0);
    Quaternion r = q * q.conjugate();
    GK_ASSERT_TRUE(r.fuzzyEquals(Quaternion::identity(), 1e-9));
}

// ── Rotation ─────────────────────────────────────────────────────────────────
GK_TEST(Quaternion, Rotate90AboutZ)
{
    Quaternion q = Quaternion::fromAxisAngle(Vec3::unitZ(), kPi / 2.0);
    Vec3       r = q.rotate(Vec3::unitX());
    GK_ASSERT_TRUE(r.fuzzyEquals(Vec3::unitY(), 1e-9));
}

GK_TEST(Quaternion, RotatePreservesNorm)
{
    Quaternion q = Quaternion::fromAxisAngle(Vec3{1,2,3}.normalized(), kPi / 5.0);
    Vec3       v{4,5,6};
    Vec3       r = q.rotate(v);
    GK_ASSERT_NEAR(r.norm(), v.norm(), 1e-9);
}

GK_TEST(Quaternion, ToMatrixConsistency)
{
    Quaternion q  = Quaternion::fromAxisAngle(Vec3::unitY(), kPi / 4.0);
    Mat3x3     M  = q.toMatrix();
    Vec3       v{1,0,1};
    Vec3       vq = q.rotate(v);
    Vec3       vM = M * v;
    GK_ASSERT_TRUE(vq.fuzzyEquals(vM, 1e-9));
}

// ── Norm & normalization ──────────────────────────────────────────────────────
GK_TEST(Quaternion, UnitNorm)
{
    Quaternion q = Quaternion::fromAxisAngle(Vec3::unitX(), 1.23);
    GK_ASSERT_NEAR(q.norm(), 1.0, 1e-12);
}

GK_TEST(Quaternion, NormalizedOfNonUnit)
{
    Quaternion q{1,2,3,4};
    Quaternion n = q.normalized();
    GK_ASSERT_NEAR(n.norm(), 1.0, 1e-12);
}

// ── Comparison ──────────────────────────────────────────────────────────────
GK_TEST(Quaternion, FuzzyEqualsNegation)
{
    Quaternion q = Quaternion::fromAxisAngle(Vec3::unitZ(), kPi / 6.0);
    Quaternion n{-q.x, -q.y, -q.z, -q.w};
    // -q represents the same rotation
    GK_ASSERT_TRUE(q.fuzzyEquals(n));
}

// ── Serialization ───────────────────────────────────────────────────────────
GK_TEST(Quaternion, ToString)
{
    std::string s = Quaternion::identity().toString();
    GK_ASSERT_TRUE(s.find("\"w\"") != std::string::npos);
}
