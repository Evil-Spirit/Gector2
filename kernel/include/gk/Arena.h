#pragma once
// Iteration 1.4 — Arena: bump-pointer allocator for transient geometry objects.
//
// Allocations are O(1) (bump of a pointer).  There is no per-object free;
// call reset() to reclaim all memory at once (objects are NOT destructed —
// callers must manually call destructors for non-trivial types if needed).
//
// Usage:
//   gk::Arena arena;
//   MyPod* p = arena.construct<MyPod>(42);
//   arena.reset(); // reclaim all memory

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <vector>

namespace gk {

class Arena
{
public:
    static constexpr std::size_t kDefaultBlockSize = 64 * 1024; ///< 64 KiB default block.

    explicit Arena(std::size_t blockSize = kDefaultBlockSize) noexcept
        : blockSize_(blockSize)
    {
    }

    // Non-copyable, movable.
    Arena(const Arena&) = delete;
    Arena& operator=(const Arena&) = delete;

    Arena(Arena&& other) noexcept
        : blocks_(std::move(other.blocks_))
        , blockSize_(other.blockSize_)
        , totalAllocated_(other.totalAllocated_)
    {
        other.totalAllocated_ = 0;
    }

    // ── Allocation ────────────────────────────────────────────────────────────

    /// Allocate 'size' bytes with at least 'alignment' alignment.
    /// Returns non-null; throws std::bad_alloc on OOM.
    void* allocate(std::size_t size, std::size_t alignment = alignof(std::max_align_t))
    {
        if (size == 0) size = 1;

        // Try to satisfy from the current (last) block.
        if (!blocks_.empty())
        {
            void* ptr = alignedAllocFromBlock(blocks_.back(), size, alignment);
            if (ptr) return ptr;
        }

        // Allocate a new block large enough for this request.
        std::size_t blockSize = std::max(blockSize_, size + alignment);
        blocks_.emplace_back(Block{std::make_unique<char[]>(blockSize), 0, blockSize});
        totalAllocated_ += blockSize;

        void* ptr = alignedAllocFromBlock(blocks_.back(), size, alignment);
        if (!ptr) throw std::bad_alloc{};
        return ptr;
    }

    /// Placement-construct a T in the arena and return its pointer.
    template<typename T, typename... Args>
    T* construct(Args&&... args)
    {
        void* mem = allocate(sizeof(T), alignof(T));
        return new (mem) T(std::forward<Args>(args)...);
    }

    // ── Reset / stats ─────────────────────────────────────────────────────────

    /// Resets all bump pointers to zero (keeps the underlying memory).
    /// Does NOT call destructors.
    void reset() noexcept
    {
        for (auto& b : blocks_)
            b.used = 0;
    }

    /// Total bytes in all allocated blocks (capacity, not necessarily used).
    std::size_t capacity() const noexcept { return totalAllocated_; }

    /// Number of underlying memory blocks.
    std::size_t blockCount() const noexcept { return blocks_.size(); }

private:
    struct Block
    {
        std::unique_ptr<char[]> data;
        std::size_t             used{0};
        std::size_t             size{0};
    };

    std::vector<Block> blocks_;
    std::size_t        blockSize_{kDefaultBlockSize};
    std::size_t        totalAllocated_{0};

    /// Try to bump-allocate 'size' bytes from 'block'; return nullptr if it
    /// doesn't fit.
    static void* alignedAllocFromBlock(Block& block, std::size_t size,
                                        std::size_t alignment) noexcept
    {
        void*       ptr       = block.data.get() + block.used;
        std::size_t remaining = block.size - block.used;

        if (std::align(alignment, size, ptr, remaining))
        {
            block.used = static_cast<std::size_t>(
                static_cast<char*>(ptr) - block.data.get()) + size;
            return ptr;
        }
        return nullptr;
    }
};

} // namespace gk
