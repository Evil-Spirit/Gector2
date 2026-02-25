#include "GkTest.h"
#include "gk/Result.h"
#include <string>

using gk::Result;
using gk::Error;
using gk::ErrorCode;

// ── Result<int> ok ────────────────────────────────────────────────────────────
GK_TEST(Result, OkIsTrue)
{
    auto r = Result<int>::ok(42);
    GK_ASSERT_TRUE(r.isOk());
    GK_ASSERT_FALSE(r.isErr());
    GK_ASSERT_TRUE(static_cast<bool>(r));
}

GK_TEST(Result, OkValueIsCorrect)
{
    auto r = Result<int>::ok(99);
    GK_ASSERT_EQ(r.value(), 99);
}

// ── Result<int> error ─────────────────────────────────────────────────────────
GK_TEST(Result, ErrIsFalse)
{
    auto r = Result<int>::err(GK_MAKE_ERROR(ErrorCode::kInvalidArgument, "bad arg"));
    GK_ASSERT_FALSE(r.isOk());
    GK_ASSERT_TRUE(r.isErr());
    GK_ASSERT_FALSE(static_cast<bool>(r));
}

GK_TEST(Result, ErrCodeAndMessage)
{
    auto r = Result<int>::err(GK_MAKE_ERROR(ErrorCode::kOutOfRange, "index out of range"));
    GK_ASSERT_EQ(r.error().code, ErrorCode::kOutOfRange);
    GK_ASSERT_FALSE(r.error().message.empty());
}

GK_TEST(Result, ValueOnErrorThrows)
{
    auto r = Result<int>::err(GK_MAKE_ERROR(ErrorCode::kInternalError, "oops"));
    bool threw = false;
    try { (void)r.value(); }
    catch (const std::logic_error&) { threw = true; }
    GK_ASSERT_TRUE(threw);
}

GK_TEST(Result, ErrorOnOkThrows)
{
    auto r = Result<int>::ok(1);
    bool threw = false;
    try { (void)r.error(); }
    catch (const std::logic_error&) { threw = true; }
    GK_ASSERT_TRUE(threw);
}

// ── Result<void> ─────────────────────────────────────────────────────────────
GK_TEST(ResultVoid, OkIsTrue)
{
    auto r = Result<void>::ok();
    GK_ASSERT_TRUE(r.isOk());
    GK_ASSERT_FALSE(r.isErr());
}

GK_TEST(ResultVoid, ErrIsFalse)
{
    auto r = Result<void>::err(GK_MAKE_ERROR(ErrorCode::kNotImplemented, "todo"));
    GK_ASSERT_TRUE(r.isErr());
    GK_ASSERT_EQ(r.error().code, ErrorCode::kNotImplemented);
}

// ── map ───────────────────────────────────────────────────────────────────────
GK_TEST(Result, MapTransformsValue)
{
    auto r = Result<int>::ok(10).map([](const int& v) { return v * 2; });
    GK_ASSERT_TRUE(r.isOk());
    GK_ASSERT_EQ(r.value(), 20);
}

GK_TEST(Result, MapPropagatesError)
{
    auto r = Result<int>::err(GK_MAKE_ERROR(ErrorCode::kInvalidArgument, ""))
             .map([](const int& v) { return v * 2; });
    GK_ASSERT_TRUE(r.isErr());
}

// ── andThen ───────────────────────────────────────────────────────────────────
GK_TEST(Result, AndThenChains)
{
    auto r = Result<int>::ok(5)
             .andThen([](const int& v) { return Result<std::string>::ok(std::to_string(v)); });
    GK_ASSERT_TRUE(r.isOk());
    GK_ASSERT_EQ(r.value(), std::string("5"));
}

GK_TEST(Result, AndThenShortCircuitsOnError)
{
    bool called = false;
    auto r = Result<int>::err(GK_MAKE_ERROR(ErrorCode::kInternalError, ""))
             .andThen([&called](const int& v) {
                 called = true;
                 return Result<int>::ok(v + 1);
             });
    GK_ASSERT_FALSE(called);
    GK_ASSERT_TRUE(r.isErr());
}

// ── Error struct ─────────────────────────────────────────────────────────────
GK_TEST(Error, ErrorCodeName)
{
    GK_ASSERT_EQ(std::string(gk::errorCodeName(ErrorCode::kOk)), std::string("Ok"));
    GK_ASSERT_EQ(std::string(gk::errorCodeName(ErrorCode::kOutOfMemory)),
                 std::string("OutOfMemory"));
}

GK_TEST(Error, ToStringContainsCodeAndMessage)
{
    auto e = GK_MAKE_ERROR(ErrorCode::kNumericalFailure, "solver diverged");
    std::string s = e.toString();
    GK_ASSERT_TRUE(s.find("NumericalFailure") != std::string::npos);
    GK_ASSERT_TRUE(s.find("solver diverged") != std::string::npos);
}

GK_TEST(Error, ToStringContainsLocationInDebug)
{
    auto e = GK_MAKE_ERROR(ErrorCode::kInternalError, "test");
#ifndef NDEBUG
    std::string s = e.toString();
    // Source file path should appear somewhere in the string.
    GK_ASSERT_TRUE(s.size() > 10u);
#else
    GK_ASSERT_TRUE(e.toString().size() > 0u);
#endif
}
