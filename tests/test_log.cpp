#include "GkTest.h"
#include "gk/Log.h"
#include <string>
#include <vector>

// Helper: collect log messages into a vector for inspection.
struct LogCapture
{
    struct Entry
    {
        gk::LogLevel level;
        std::string  file;
        int          line;
        std::string  msg;
    };
    std::vector<Entry> entries;

    void install()
    {
        entries.clear();
        gk::Logger::setSink([this](gk::LogLevel lvl, const char* file, int line,
                                    const std::string& msg) {
            entries.push_back({lvl, file ? file : "", line, msg});
        });
    }
    void uninstall() { gk::Logger::clearSink(); }
};

// ── logLevelName ──────────────────────────────────────────────────────────────
GK_TEST(Logger, LevelNames)
{
    GK_ASSERT_EQ(std::string(gk::logLevelName(gk::LogLevel::kTrace)),   std::string("TRACE"));
    GK_ASSERT_EQ(std::string(gk::logLevelName(gk::LogLevel::kDebug)),   std::string("DEBUG"));
    GK_ASSERT_EQ(std::string(gk::logLevelName(gk::LogLevel::kInfo)),    std::string("INFO"));
    GK_ASSERT_EQ(std::string(gk::logLevelName(gk::LogLevel::kWarning)), std::string("WARNING"));
    GK_ASSERT_EQ(std::string(gk::logLevelName(gk::LogLevel::kError)),   std::string("ERROR"));
    GK_ASSERT_EQ(std::string(gk::logLevelName(gk::LogLevel::kFatal)),   std::string("FATAL"));
}

// ── Custom sink receives messages ─────────────────────────────────────────────
GK_TEST(Logger, CustomSinkReceivesMessage)
{
    LogCapture cap;
    cap.install();
    gk::Logger::setMinLevel(gk::LogLevel::kTrace); // capture everything

    GK_LOG_INFO("hello from test");

    cap.uninstall();
    gk::Logger::setMinLevel(gk::LogLevel::kInfo); // restore

    GK_ASSERT_EQ(cap.entries.size(), 1u);
    GK_ASSERT_EQ(cap.entries[0].level, gk::LogLevel::kInfo);
    GK_ASSERT_EQ(cap.entries[0].msg, std::string("hello from test"));
}

GK_TEST(Logger, SinkReceivesFileAndLine)
{
    LogCapture cap;
    cap.install();
    gk::Logger::setMinLevel(gk::LogLevel::kTrace);

    GK_LOG_DEBUG("debug msg");

    cap.uninstall();
    gk::Logger::setMinLevel(gk::LogLevel::kInfo);

    GK_ASSERT_EQ(cap.entries.size(), 1u);
    GK_ASSERT_FALSE(cap.entries[0].file.empty());
    GK_ASSERT_TRUE(cap.entries[0].line > 0);
}

// ── Log level filtering ───────────────────────────────────────────────────────
GK_TEST(Logger, MessagesFilteredBelowMinLevel)
{
    LogCapture cap;
    cap.install();
    gk::Logger::setMinLevel(gk::LogLevel::kWarning);

    GK_LOG_DEBUG("should be filtered");
    GK_LOG_INFO("should be filtered");
    GK_LOG_WARNING("should appear");
    GK_LOG_ERROR("should appear");

    cap.uninstall();
    gk::Logger::setMinLevel(gk::LogLevel::kInfo);

    GK_ASSERT_EQ(cap.entries.size(), 2u);
    GK_ASSERT_EQ(cap.entries[0].level, gk::LogLevel::kWarning);
    GK_ASSERT_EQ(cap.entries[1].level, gk::LogLevel::kError);
}

// ── GK_LOG_* macros send all levels ──────────────────────────────────────────
GK_TEST(Logger, AllMacroLevels)
{
    LogCapture cap;
    cap.install();
    gk::Logger::setMinLevel(gk::LogLevel::kTrace);

    GK_LOG_TRACE("t");
    GK_LOG_DEBUG("d");
    GK_LOG_INFO("i");
    GK_LOG_WARNING("w");
    GK_LOG_ERROR("e");
    GK_LOG_FATAL("f");

    cap.uninstall();
    gk::Logger::setMinLevel(gk::LogLevel::kInfo);

    GK_ASSERT_EQ(cap.entries.size(), 6u);
}

// ── clearSink restores default behaviour (no crash) ───────────────────────────
GK_TEST(Logger, ClearSinkIsHarmless)
{
    gk::Logger::setSink([](gk::LogLevel, const char*, int, const std::string&){});
    gk::Logger::clearSink();
    // After clearing, logging to stderr should not crash.
    // (We can't easily capture stderr here, so just verify no exception/crash.)
    gk::Logger::log(gk::LogLevel::kInfo, __FILE__, __LINE__, "post-clear message");
    GK_ASSERT_TRUE(true); // reached here without crashing
}

// ── minLevel accessor ─────────────────────────────────────────────────────────
GK_TEST(Logger, MinLevelAccessor)
{
    gk::Logger::setMinLevel(gk::LogLevel::kError);
    GK_ASSERT_EQ(gk::Logger::minLevel(), gk::LogLevel::kError);
    gk::Logger::setMinLevel(gk::LogLevel::kInfo); // restore default
}
