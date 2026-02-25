#pragma once
// GkTest.h — thin Google Test wrapper.
// All GK_* macros are preserved so existing test sources compile unchanged.
// Visual Studio Test Explorer discovers every TEST() case automatically via
// the built-in Google Test adapter when tests are linked with GTest::gtest_main.

#ifdef _MSC_VER
// Suppress C4127 "conditional expression is constant" triggered by gtest
// internals and by do{…}while(false) patterns.
#pragma warning(disable : 4127)
#endif

#include <gtest/gtest.h>

// ── Test-case declaration ─────────────────────────────────────────────────────
#define GK_TEST(Suite, Name) TEST(Suite, Name)

// ── Assertion macros (non-fatal — report failure and continue) ────────────────
#define GK_ASSERT(expr)              EXPECT_TRUE(expr)
#define GK_ASSERT_MSG(expr, msg)     EXPECT_TRUE(expr) << (msg)
#define GK_ASSERT_EQ(a, b)           EXPECT_EQ(a, b)
#define GK_ASSERT_NE(a, b)           EXPECT_NE(a, b)
#define GK_ASSERT_NEAR(a, b, tol)    EXPECT_NEAR(a, b, tol)
#define GK_ASSERT_TRUE(expr)         EXPECT_TRUE(expr)
#define GK_ASSERT_FALSE(expr)        EXPECT_FALSE(expr)

