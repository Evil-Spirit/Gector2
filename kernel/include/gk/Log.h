#pragma once
// Iteration 1.5 — Logging facade with pluggable back-end.
//
// Usage:
//   gk::Logger::setSink([](gk::LogLevel lvl, const char* file, int line,
//                           const std::string& msg) { … });
//   GK_LOG_INFO("geometry loaded");
//   GK_LOG_ERROR("solver diverged");

#include <functional>
#include <string>

namespace gk {

// ── Log level ──────────────────────────────────────────────────────────────────
enum class LogLevel : int
{
    kTrace   = 0,
    kDebug   = 1,
    kInfo    = 2,
    kWarning = 3,
    kError   = 4,
    kFatal   = 5,
};

inline const char* logLevelName(LogLevel level) noexcept
{
    switch (level)
    {
    case LogLevel::kTrace:   return "TRACE";
    case LogLevel::kDebug:   return "DEBUG";
    case LogLevel::kInfo:    return "INFO";
    case LogLevel::kWarning: return "WARNING";
    case LogLevel::kError:   return "ERROR";
    case LogLevel::kFatal:   return "FATAL";
    default:                 return "UNKNOWN";
    }
}

// ── Sink type ──────────────────────────────────────────────────────────────────
using LogSink = std::function<void(LogLevel level,
                                   const char* file,
                                   int         line,
                                   const std::string& msg)>;

// ── Logger singleton ───────────────────────────────────────────────────────────

/// Thread-safe log dispatcher.
/// Callers install a custom sink; the default sink writes to stderr.
class Logger
{
public:
    /// Replace the current sink.  Pass nullptr to restore the default stderr sink.
    static void setSink(LogSink sink) noexcept
    {
        state().sink = std::move(sink);
    }

    /// Remove the custom sink (messages go to stderr again).
    static void clearSink() noexcept { state().sink = nullptr; }

    /// Set the minimum level; messages below this level are silently dropped.
    static void setMinLevel(LogLevel level) noexcept { state().minLevel = level; }

    /// Current minimum level.
    static LogLevel minLevel() noexcept { return state().minLevel; }

    /// Dispatch a log message.  Called by GK_LOG_* macros.
    static void log(LogLevel level, const char* file, int line,
                    const std::string& msg) noexcept
    {
        auto& s = state();
        if (level < s.minLevel) return;

        if (s.sink)
        {
            try { s.sink(level, file, line, msg); }
            catch (...) { /* sinks must not throw */ }
        }
        else
        {
            defaultSink(level, file, line, msg);
        }
    }

private:
    struct State
    {
        LogSink  sink;
        LogLevel minLevel{LogLevel::kInfo};
    };

    static State& state() noexcept
    {
        static State s;
        return s;
    }

    /// Built-in stderr sink (used when no custom sink is installed).
    static void defaultSink(LogLevel level, const char* file, int line,
                             const std::string& msg) noexcept;
};

} // namespace gk

// ── Implementation of defaultSink (inline, no separate .cpp needed) ──────────
#include <cstdio>

namespace gk {

inline void Logger::defaultSink(LogLevel level, const char* file, int line,
                                 const std::string& msg) noexcept
{
    std::fprintf(stderr, "[%s] %s:%d: %s\n",
                 logLevelName(level), file ? file : "", line, msg.c_str());
}

} // namespace gk

// ── Macros ────────────────────────────────────────────────────────────────────
#define GK_LOG_TRACE(msg)   gk::Logger::log(gk::LogLevel::kTrace,   __FILE__, __LINE__, (msg))
#define GK_LOG_DEBUG(msg)   gk::Logger::log(gk::LogLevel::kDebug,   __FILE__, __LINE__, (msg))
#define GK_LOG_INFO(msg)    gk::Logger::log(gk::LogLevel::kInfo,     __FILE__, __LINE__, (msg))
#define GK_LOG_WARNING(msg) gk::Logger::log(gk::LogLevel::kWarning,  __FILE__, __LINE__, (msg))
#define GK_LOG_ERROR(msg)   gk::Logger::log(gk::LogLevel::kError,    __FILE__, __LINE__, (msg))
#define GK_LOG_FATAL(msg)   gk::Logger::log(gk::LogLevel::kFatal,    __FILE__, __LINE__, (msg))
