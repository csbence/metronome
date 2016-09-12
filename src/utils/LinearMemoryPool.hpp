#ifndef METRONOME_LINEARMEMORYPOOL_HPP
#define METRONOME_LINEARMEMORYPOOL_HPP

#include <cstdio>
#include <memory>
namespace metronome {

template <typename T, std::size_t N>
class StaticVector {
    // properly aligned uninitialized storage for N T's
    typename std::aligned_storage<sizeof(T), alignof(T)>::type data[N];
    std::size_t size = 0;

public:
    // Create an object in aligned storage
    template <typename... Args>
    T* construct(Args&&... args) {
        if (size >= N) // possible error handling
            throw std::bad_alloc{};
        T* const t = new (data + size) T(std::forward<Args>(args)...);
        ++size;

        return t;
    }

    // Access an object in aligned storage
    const T& operator[](std::size_t pos) const { return *reinterpret_cast<const T*>(data + pos); }

    // Delete objects from aligned storage
    ~StaticVector() {
        for (std::size_t pos = 0; pos < size; ++pos) {
            reinterpret_cast<const T*>(data + pos)->~T();
        }
    }
};
}

#endif // METRONOME_LINEARMEMORYPOOL_HPP
