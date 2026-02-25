#include "GkTest.h"
#include "gk/Arena.h"
#include <cstddef>

// ── Basic allocation ──────────────────────────────────────────────────────────
GK_TEST(Arena, AllocateReturnsNonNull)
{
    gk::Arena arena;
    void* p = arena.allocate(16);
    GK_ASSERT_TRUE(p != nullptr);
}

GK_TEST(Arena, MultipleAllocationsDoNotOverlap)
{
    gk::Arena arena;
    int* a = arena.construct<int>(1);
    int* b = arena.construct<int>(2);
    GK_ASSERT_TRUE(a != b);
    GK_ASSERT_EQ(*a, 1);
    GK_ASSERT_EQ(*b, 2);
}

GK_TEST(Arena, ConstructCallsConstructor)
{
    gk::Arena arena;
    struct Point { double x, y; Point(double x_, double y_) : x(x_), y(y_) {} };
    Point* p = arena.construct<Point>(3.0, 4.0);
    GK_ASSERT_NEAR(p->x, 3.0, 1e-12);
    GK_ASSERT_NEAR(p->y, 4.0, 1e-12);
}

// ── Alignment ─────────────────────────────────────────────────────────────────
GK_TEST(Arena, AllocationsAreAligned)
{
    gk::Arena arena;
    // double requires 8-byte alignment
    double* d = arena.construct<double>(1.0);
    GK_ASSERT_EQ(reinterpret_cast<std::uintptr_t>(d) % alignof(double), 0u);
    // char should work too
    char* c = arena.construct<char>('A');
    GK_ASSERT_EQ(*c, 'A');
}

GK_TEST(Arena, LargeAlignmentHonoured)
{
    struct alignas(64) Aligned64 { char data[64]; };
    gk::Arena arena;
    Aligned64* p = arena.construct<Aligned64>();
    GK_ASSERT_EQ(reinterpret_cast<std::uintptr_t>(p) % 64u, 0u);
}

// ── Reset ─────────────────────────────────────────────────────────────────────
GK_TEST(Arena, ResetAllowsReuse)
{
    gk::Arena arena(256);
    int* a = arena.construct<int>(10);
    *a = 42;
    arena.reset();
    // After reset the bump pointer goes back to 0; a new allocation may return
    // the same address (implementation-defined) but must succeed.
    int* b = arena.construct<int>(99);
    GK_ASSERT_TRUE(b != nullptr);
    GK_ASSERT_EQ(*b, 99);
}

// ── Capacity / stats ─────────────────────────────────────────────────────────
GK_TEST(Arena, CapacityIsAtLeastBlockSize)
{
    gk::Arena arena(1024);
    arena.construct<char>('x');
    GK_ASSERT_TRUE(arena.capacity() >= 1024u);
}

GK_TEST(Arena, NewBlockAllocatedWhenCurrentFull)
{
    // Small block size to force a second block.
    gk::Arena arena(32);
    for (int i = 0; i < 10; ++i)
        arena.construct<double>(static_cast<double>(i));
    GK_ASSERT_TRUE(arena.blockCount() >= 2u);
}

// ── Zero-size allocation ──────────────────────────────────────────────────────
GK_TEST(Arena, ZeroSizeAllocationSucceeds)
{
    gk::Arena arena;
    void* p = arena.allocate(0);
    GK_ASSERT_TRUE(p != nullptr);
}
