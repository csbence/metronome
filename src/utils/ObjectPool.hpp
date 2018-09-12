#ifndef METRONOME_LINEARMEMORYPOOL_HPP
#define METRONOME_LINEARMEMORYPOOL_HPP

#include <cstdio>
#include <memory>
namespace metronome {

template <typename T, std::size_t N>
class ObjectPool {
  using Storage = typename std::aligned_storage<sizeof(T), alignof(T)>::type;

 public:
  ObjectPool() : storage(new Storage[N]) {}

  ObjectPool(const ObjectPool&) = delete;

  ~ObjectPool() {
    purge();
    delete[] storage;
  }

  // Create an object in aligned storage
  template <typename... Args>
  T* construct(Args&&... args) {
    if (size >= N)  // possible error handling
      throw std::bad_alloc{};
    T* const t = new (storage + size) T(std::forward<Args>(args)...);
    ++size;

    return t;
  }

  void purge() {
    for (std::size_t i = 0; i < size; ++i) {
      reinterpret_cast<const T*>(storage + i)->~T();
    }

    delete[] storage;
  }

  // Access an object in aligned storage
  T* operator[](std::size_t index) const {
    return reinterpret_cast<const T*>(storage + index);
  }

  bool empty() const { return size == 0; }

 private:
  Storage* storage;
  std::size_t size = 0;
};
}  // namespace metronome

#endif  // METRONOME_LINEARMEMORYPOOL_HPP
