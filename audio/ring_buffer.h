// audio/ring_buffer.h — lock-free SPSC ring
#pragma once
#include <atomic>
#include <vector>
#include <cstring>

namespace ice {

template <typename T>
class SpscRing {
public:
    void initialize(size_t capacity) {
        m_buf.assign(capacity, T{});
        m_cap = capacity;
        m_head = 0;
        m_tail = 0;
    }
    // producer thread
    bool push(const T *items, size_t n) {
        const size_t head = m_head.load(std::memory_order_relaxed);
        const size_t used = head - m_tail.load(std::memory_order_acquire);
        if (used + n > m_cap) return false;
        for (size_t i = 0; i < n; ++i) {
            m_buf[(head + i) % m_cap] = items[i];
        }
        m_head.store(head + n, std::memory_order_release);
        return true;
    }
    // consumer thread
    size_t pop(T *items, size_t n) {
        const size_t tail = m_tail.load(std::memory_order_relaxed);
        const size_t avail = m_head.load(std::memory_order_acquire) - tail;
        const size_t k = std::min(n, avail);
        for (size_t i = 0; i < k; ++i) {
            items[i] = m_buf[(tail + i) % m_cap];
        }
        m_tail.store(tail + k, std::memory_order_release);
        return k;
    }
    size_t size() const {
        return m_head.load(std::memory_order_acquire) - m_tail.load(std::memory_order_acquire);
    }
    size_t capacity() const { return m_cap; }

private:
    std::vector<T> m_buf;
    size_t m_cap = 0;
    std::atomic<size_t> m_head{0}, m_tail{0};
};

} // namespace ice
