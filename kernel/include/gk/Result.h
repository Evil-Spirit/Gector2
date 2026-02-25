#pragma once
// Iteration 1.5 — Result<T, Error>: exception-free error propagation monad.
//
// Usage:
//   gk::Result<double> divide(double a, double b) {
//       if (b == 0.0)
//           return gk::Result<double>::err(GK_MAKE_ERROR(gk::ErrorCode::kInvalidArgument, "b=0"));
//       return gk::Result<double>::ok(a / b);
//   }
//   auto r = divide(10.0, 2.0);
//   if (r) double v = r.value();

#include "gk/Error.h"
#include <stdexcept>
#include <utility>
#include <variant>

namespace gk {

// ── Primary template ──────────────────────────────────────────────────────────

template<typename T>
class Result
{
    // Wrap T and Error in distinguishable tag types to avoid ambiguity when
    // T happens to be Error.
    struct OkTag  { T     value; };
    struct ErrTag { Error error; };

public:
    // ── Factory ───────────────────────────────────────────────────────────────
    static Result ok(T value)   { return Result{OkTag{std::move(value)}}; }
    static Result err(Error e)  { return Result{ErrTag{std::move(e)}};    }

    // ── Observers ─────────────────────────────────────────────────────────────
    bool isOk()  const noexcept { return std::holds_alternative<OkTag>(storage_);  }
    bool isErr() const noexcept { return std::holds_alternative<ErrTag>(storage_); }

    /// Implicit bool: true when the result is ok.
    explicit operator bool() const noexcept { return isOk(); }

    /// Access the success value.  Throws std::logic_error if this is an error.
    T& value()
    {
        if (!isOk())
            throw std::logic_error("Result::value() called on an error result: " +
                                   error().toString());
        return std::get<OkTag>(storage_).value;
    }
    const T& value() const
    {
        if (!isOk())
            throw std::logic_error("Result::value() called on an error result: " +
                                   error().toString());
        return std::get<OkTag>(storage_).value;
    }

    /// Access the error.  Throws std::logic_error if this is ok.
    Error& error()
    {
        if (!isErr())
            throw std::logic_error("Result::error() called on a successful result");
        return std::get<ErrTag>(storage_).error;
    }
    const Error& error() const
    {
        if (!isErr())
            throw std::logic_error("Result::error() called on a successful result");
        return std::get<ErrTag>(storage_).error;
    }

    // ── Monadic operations ────────────────────────────────────────────────────

    /// map: transform the value if ok, propagate the error otherwise.
    template<typename F>
    auto map(F&& f) const -> Result<decltype(f(std::declval<const T&>()))>
    {
        using U = decltype(f(std::declval<const T&>()));
        if (isOk())  return Result<U>::ok(f(value()));
        if (isErr()) return Result<U>::err(error());
        return Result<U>::err(GK_MAKE_ERROR(ErrorCode::kInternalError, "empty Result"));
    }

    /// andThen: chain a function that itself returns a Result.
    template<typename F>
    auto andThen(F&& f) const -> decltype(f(std::declval<const T&>()))
    {
        using R = decltype(f(std::declval<const T&>()));
        if (isOk())  return f(value());
        if (isErr()) return R::err(error());
        return R::err(GK_MAKE_ERROR(ErrorCode::kInternalError, "empty Result"));
    }

private:
    std::variant<OkTag, ErrTag> storage_;
    explicit Result(OkTag  t) : storage_(std::move(t)) {}
    explicit Result(ErrTag t) : storage_(std::move(t)) {}
};

// ── void specialization ───────────────────────────────────────────────────────

template<>
class Result<void>
{
public:
    static Result ok()         { return Result{true,  Error{}}; }
    static Result err(Error e) { return Result{false, std::move(e)}; }

    bool isOk()  const noexcept { return isOk_;  }
    bool isErr() const noexcept { return !isOk_; }
    explicit operator bool() const noexcept { return isOk_; }

    Error& error()
    {
        if (isOk_)
            throw std::logic_error("Result::error() called on a successful result");
        return error_;
    }
    const Error& error() const
    {
        if (isOk_)
            throw std::logic_error("Result::error() called on a successful result");
        return error_;
    }

private:
    bool  isOk_{true};
    Error error_{};
    Result(bool ok, Error e) : isOk_(ok), error_(std::move(e)) {}
};

} // namespace gk
