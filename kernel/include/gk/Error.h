#pragma once
// Iteration 1.5 — Structured error codes and Error type.

#include <string>

namespace gk {

// ── Error codes ───────────────────────────────────────────────────────────────

enum class ErrorCode : int
{
    kOk                = 0,
    kInvalidArgument   = 1,
    kOutOfRange        = 2,
    kOutOfMemory       = 3,
    kNotImplemented    = 4,
    kInternalError     = 5,
    kNumericalFailure  = 6,  ///< Solver / geometric algorithm failed to converge.
    kGeometryInvalid   = 7,  ///< Malformed geometric input.
    kIoError           = 8,  ///< File read / write failure.
};

/// Human-readable name for an error code.
inline const char* errorCodeName(ErrorCode code) noexcept
{
    switch (code)
    {
    case ErrorCode::kOk:               return "Ok";
    case ErrorCode::kInvalidArgument:  return "InvalidArgument";
    case ErrorCode::kOutOfRange:       return "OutOfRange";
    case ErrorCode::kOutOfMemory:      return "OutOfMemory";
    case ErrorCode::kNotImplemented:   return "NotImplemented";
    case ErrorCode::kInternalError:    return "InternalError";
    case ErrorCode::kNumericalFailure: return "NumericalFailure";
    case ErrorCode::kGeometryInvalid:  return "GeometryInvalid";
    case ErrorCode::kIoError:          return "IoError";
    default:                           return "UnknownError";
    }
}

// ── Error struct ──────────────────────────────────────────────────────────────

/// Carries a structured error code plus a human-readable message.
/// In debug builds it also captures the source location.
struct Error
{
    ErrorCode   code{ErrorCode::kInternalError};
    std::string message;

#ifndef NDEBUG
    const char* file{nullptr};
    int         line{0};
#endif

    // ── Factory helpers ───────────────────────────────────────────────────────

    /// Construct with code + message (+ source location in debug).
    static Error make(ErrorCode code, std::string msg
#ifndef NDEBUG
                      , const char* file = nullptr, int line = 0
#endif
                      ) noexcept
    {
        Error e;
        e.code    = code;
        e.message = std::move(msg);
#ifndef NDEBUG
        e.file = file;
        e.line = line;
#endif
        return e;
    }

    std::string toString() const
    {
        std::string s = std::string("[") + errorCodeName(code) + "] " + message;
#ifndef NDEBUG
        if (file)
            s += " (" + std::string(file) + ":" + std::to_string(line) + ")";
#endif
        return s;
    }
};

// ── Convenience macros ────────────────────────────────────────────────────────

#ifndef NDEBUG
/// Create an Error with automatic source-location capture (debug builds).
#define GK_MAKE_ERROR(code, msg) \
    gk::Error::make((code), (msg), __FILE__, __LINE__)
#else
#define GK_MAKE_ERROR(code, msg) \
    gk::Error::make((code), (msg))
#endif

} // namespace gk
