#include "GkTest.h"
#include "gk/EntityId.h"
#include <set>
#include <string>

GK_TEST(EntityId, DefaultIsInvalid)
{
    gk::EntityId id;
    GK_ASSERT_FALSE(id.isValid());
    GK_ASSERT_EQ(id.value(), gk::EntityId::IdType{0});
}

GK_TEST(EntityId, InvalidSentinelIsInvalid)
{
    GK_ASSERT_FALSE(gk::EntityId::invalid().isValid());
}

GK_TEST(EntityId, GenerateReturnsValidId)
{
    gk::EntityId id = gk::EntityId::generate();
    GK_ASSERT_TRUE(id.isValid());
}

GK_TEST(EntityId, GeneratedIdsAreUnique)
{
    std::set<gk::EntityId::IdType> seen;
    for (int i = 0; i < 1000; ++i)
    {
        gk::EntityId id = gk::EntityId::generate();
        GK_ASSERT_TRUE(id.isValid());
        GK_ASSERT_TRUE(seen.find(id.value()) == seen.end());
        seen.insert(id.value());
    }
}

GK_TEST(EntityId, Equality)
{
    gk::EntityId a = gk::EntityId::generate();
    gk::EntityId b = a;
    GK_ASSERT_EQ(a, b);

    gk::EntityId c = gk::EntityId::generate();
    GK_ASSERT_NE(a, c);
}

GK_TEST(EntityId, LessThanOrdersById)
{
    gk::EntityId a = gk::EntityId::generate();
    gk::EntityId b = gk::EntityId::generate();
    // Sequential generation means a < b.
    GK_ASSERT_TRUE(a < b);
}

GK_TEST(EntityId, InvalidLessThanValid)
{
    gk::EntityId inv = gk::EntityId::invalid();
    gk::EntityId val = gk::EntityId::generate();
    GK_ASSERT_TRUE(inv < val);
}

GK_TEST(EntityId, ToString)
{
    gk::EntityId id = gk::EntityId::generate();
    std::string s = id.toString();
    GK_ASSERT_TRUE(s.find("EntityId(") != std::string::npos);
}
