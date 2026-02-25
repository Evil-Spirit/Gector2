// Smoke test — verifies that the build system and test runner work end-to-end.
#include "GkTest.h"

GK_TEST(Smoke, AlwaysPasses)
{
    GK_ASSERT(1 + 1 == 2);
}

GK_TEST(Smoke, AssertNear)
{
    GK_ASSERT_NEAR(1.0, 1.0 + 1e-11, 1e-10);
}

int main()
{
    return gk::test::runAll();
}
