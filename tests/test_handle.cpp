#include "GkTest.h"
#include "gk/Handle.h"

// ── Test fixture: a ref-counted object that tracks its own lifetime ────────────
struct Widget : gk::RefCounted
{
    int& liveCount;
    int  value;

    Widget(int& counter, int v) : liveCount(counter), value(v) { ++liveCount; }
    ~Widget() override { --liveCount; }
};

// ── Handle construction & destruction ─────────────────────────────────────────
GK_TEST(Handle, DefaultIsNull)
{
    gk::Handle<Widget> h;
    GK_ASSERT_FALSE(static_cast<bool>(h));
    GK_ASSERT_EQ(h.get(), nullptr);
}

GK_TEST(Handle, OwnsObjectAfterConstruction)
{
    int live = 0;
    {
        auto h = gk::makeHandle<Widget>(live, 42);
        GK_ASSERT_EQ(live, 1);
        GK_ASSERT_TRUE(static_cast<bool>(h));
        GK_ASSERT_EQ(h->value, 42);
    }
    GK_ASSERT_EQ(live, 0); // destroyed when h goes out of scope
}

// ── Reference counting ────────────────────────────────────────────────────────
GK_TEST(Handle, CopyIncreasesRefCount)
{
    int live = 0;
    auto h1 = gk::makeHandle<Widget>(live, 1);
    GK_ASSERT_EQ(h1->refCount(), 1);

    gk::Handle<Widget> h2 = h1;
    GK_ASSERT_EQ(h1->refCount(), 2);
    GK_ASSERT_EQ(h2->refCount(), 2);
    GK_ASSERT_EQ(live, 1); // still only one object
}

GK_TEST(Handle, DestroyingCopyDecrementsRefCount)
{
    int live = 0;
    auto h1 = gk::makeHandle<Widget>(live, 1);
    {
        gk::Handle<Widget> h2 = h1;
        GK_ASSERT_EQ(h1->refCount(), 2);
    }
    GK_ASSERT_EQ(h1->refCount(), 1);
    GK_ASSERT_EQ(live, 1);
}

GK_TEST(Handle, LastHandleDestroysObject)
{
    int live = 0;
    {
        auto h1 = gk::makeHandle<Widget>(live, 1);
        {
            gk::Handle<Widget> h2 = h1;
            gk::Handle<Widget> h3 = h2;
            GK_ASSERT_EQ(h1->refCount(), 3);
        }
        GK_ASSERT_EQ(h1->refCount(), 1);
        GK_ASSERT_EQ(live, 1);
    }
    GK_ASSERT_EQ(live, 0);
}

// ── Move semantics ────────────────────────────────────────────────────────────
GK_TEST(Handle, MoveTransfersOwnership)
{
    int live = 0;
    auto h1 = gk::makeHandle<Widget>(live, 5);
    gk::Handle<Widget> h2 = std::move(h1);

    GK_ASSERT_EQ(h1.get(), nullptr); // h1 now null
    GK_ASSERT_EQ(live, 1);
    GK_ASSERT_EQ(h2->refCount(), 1);
    GK_ASSERT_EQ(h2->value, 5);
}

// ── reset ─────────────────────────────────────────────────────────────────────
GK_TEST(Handle, ResetReleasesObject)
{
    int live = 0;
    auto h = gk::makeHandle<Widget>(live, 1);
    GK_ASSERT_EQ(live, 1);
    h.reset();
    GK_ASSERT_EQ(live, 0);
    GK_ASSERT_EQ(h.get(), nullptr);
}

// ── Equality ─────────────────────────────────────────────────────────────────
GK_TEST(Handle, EqualityPointsToSameObject)
{
    int live = 0;
    auto h1 = gk::makeHandle<Widget>(live, 1);
    gk::Handle<Widget> h2 = h1;
    GK_ASSERT_TRUE(h1 == h2);

    auto h3 = gk::makeHandle<Widget>(live, 2);
    GK_ASSERT_TRUE(h1 != h3);
}

// ── Dereference ───────────────────────────────────────────────────────────────
GK_TEST(Handle, Dereference)
{
    int live = 0;
    auto h = gk::makeHandle<Widget>(live, 99);
    GK_ASSERT_EQ((*h).value, 99);
    GK_ASSERT_EQ(h->value, 99);
}
