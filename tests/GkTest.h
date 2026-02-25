#pragma once
// Minimal single-header test framework for GectorKernel.
// Usage:
//   GK_TEST(SuiteName, TestName) { GK_ASSERT(expr); GK_ASSERT_EQ(a, b); ... }
//   int main() { return gk::test::runAll(); }

// MSVC: 'do { } while(false)' in macros triggers C4127 (conditional expression
// is constant).  Disable it for this header and all translation units that
// include it — this is safe because the header is only ever included by test
// source files.
#ifdef _MSC_VER
#pragma warning(disable : 4127)
#endif

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace gk {
namespace test {

struct TestCase
{
    std::string        suite;
    std::string        name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry()
{
    static std::vector<TestCase> cases;
    return cases;
}

struct Registrar
{
    Registrar(const char* suite, const char* name, std::function<void()> fn)
    {
        registry().push_back({suite, name, std::move(fn)});
    }
};

struct TestFailure
{
    std::string message;
};

inline int runAll()
{
    int passed = 0, failed = 0;
    for (auto& tc : registry())
    {
        try
        {
            tc.fn();
            std::cout << "[PASS] " << tc.suite << "." << tc.name << "\n";
            ++passed;
        }
        catch (const TestFailure& f)
        {
            std::cout << "[FAIL] " << tc.suite << "." << tc.name << " : " << f.message << "\n";
            ++failed;
        }
        catch (const std::exception& e)
        {
            std::cout << "[FAIL] " << tc.suite << "." << tc.name << " : exception: " << e.what() << "\n";
            ++failed;
        }
        catch (...)
        {
            std::cout << "[FAIL] " << tc.suite << "." << tc.name << " : unknown exception\n";
            ++failed;
        }
    }
    std::cout << "\n" << passed << " passed, " << failed << " failed.\n";
    return (failed > 0) ? 1 : 0;
}

} // namespace test
} // namespace gk

// ── Macros ───────────────────────────────────────────────────────────────────
#define GK_TEST(Suite, Name)                                                   \
    static void gk_test_##Suite##_##Name();                                    \
    static gk::test::Registrar gk_reg_##Suite##_##Name(                        \
        #Suite, #Name, gk_test_##Suite##_##Name);                              \
    static void gk_test_##Suite##_##Name()

#define GK_ASSERT(expr)                                                        \
    do {                                                                       \
        if (!(expr)) {                                                         \
            std::ostringstream _os;                                            \
            _os << __FILE__ << ":" << __LINE__ << ": assertion failed: " #expr;\
            throw gk::test::TestFailure{_os.str()};                           \
        }                                                                      \
    } while (false)

#define GK_ASSERT_MSG(expr, msg)                                               \
    do {                                                                       \
        if (!(expr)) {                                                         \
            std::ostringstream _os;                                            \
            _os << __FILE__ << ":" << __LINE__ << ": " << msg;                \
            throw gk::test::TestFailure{_os.str()};                           \
        }                                                                      \
    } while (false)

#define GK_ASSERT_EQ(a, b) GK_ASSERT((a) == (b))
#define GK_ASSERT_NE(a, b) GK_ASSERT((a) != (b))

#define GK_ASSERT_NEAR(a, b, tol)                                              \
    do {                                                                       \
        double _a = static_cast<double>(a);                                    \
        double _b = static_cast<double>(b);                                    \
        double _t = static_cast<double>(tol);                                  \
        double _d = _a - _b;                                                   \
        if (!(_d >= -_t && _d <= _t)) {                                        \
            std::ostringstream _os;                                            \
            _os << __FILE__ << ":" << __LINE__                                 \
                << ": |" << _a << " - " << _b << "| = "                       \
                << (_d < 0 ? -_d : _d) << " > " << _t;                        \
            throw gk::test::TestFailure{_os.str()};                           \
        }                                                                      \
    } while (false)

#define GK_ASSERT_TRUE(expr)  GK_ASSERT(expr)
#define GK_ASSERT_FALSE(expr) GK_ASSERT(!(expr))
