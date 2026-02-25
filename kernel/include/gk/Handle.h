#pragma once
// Iteration 1.4 — Handle<T>: intrusive reference-counted smart pointer.
//
// Usage:
//   class MyGeom : public gk::RefCounted { … };
//   gk::Handle<MyGeom> h = gk::makeHandle<MyGeom>(…);

#include <atomic>
#include <utility>

namespace gk {

// ── RefCounted base ───────────────────────────────────────────────────────────

/// Mixin base class for objects managed by Handle<T>.
/// Deriving classes should declare a virtual destructor.
class RefCounted
{
public:
    void addRef() noexcept  { ++refCount_; }
    void release() noexcept { if (--refCount_ == 0) delete this; }

    /// Current reference count (primarily useful in tests).
    int refCount() const noexcept { return refCount_.load(std::memory_order_relaxed); }

protected:
    virtual ~RefCounted() = default;

private:
    std::atomic<int> refCount_{0};
};

// ── Handle<T> ─────────────────────────────────────────────────────────────────

/// Intrusive reference-counted smart pointer.
/// T must derive from RefCounted.
template<typename T>
class Handle
{
public:
    // ── Construction ──────────────────────────────────────────────────────────
    constexpr Handle() noexcept = default;

    /// Take ownership of a raw pointer (call addRef).
    explicit Handle(T* ptr) noexcept : ptr_(ptr) { if (ptr_) ptr_->addRef(); }

    Handle(const Handle& other) noexcept : ptr_(other.ptr_) { if (ptr_) ptr_->addRef(); }
    Handle(Handle&& other)      noexcept : ptr_(other.ptr_) { other.ptr_ = nullptr; }

    /// Allow upcasting: Handle<Derived> → Handle<Base>.
    template<typename U>
    Handle(const Handle<U>& other) noexcept : ptr_(other.get()) // NOLINT(google-explicit-constructor)
    {
        if (ptr_) ptr_->addRef();
    }

    ~Handle() { if (ptr_) ptr_->release(); }

    // ── Assignment ────────────────────────────────────────────────────────────
    Handle& operator=(Handle other) noexcept { swap(other); return *this; }

    void swap(Handle& other) noexcept { std::swap(ptr_, other.ptr_); }

    // ── Observers ─────────────────────────────────────────────────────────────
    T* get()        const noexcept { return ptr_; }
    T* operator->() const noexcept { return ptr_; }
    T& operator*()  const noexcept { return *ptr_; }

    explicit operator bool() const noexcept { return ptr_ != nullptr; }

    bool operator==(const Handle& o) const noexcept { return ptr_ == o.ptr_; }
    bool operator!=(const Handle& o) const noexcept { return ptr_ != o.ptr_; }

    // ── Modifiers ─────────────────────────────────────────────────────────────
    void reset() noexcept { Handle{}.swap(*this); }

private:
    T* ptr_{nullptr};
};

/// Convenience factory (equivalent to std::make_shared).
template<typename T, typename... Args>
Handle<T> makeHandle(Args&&... args)
{
    return Handle<T>(new T(std::forward<Args>(args)...));
}

} // namespace gk
