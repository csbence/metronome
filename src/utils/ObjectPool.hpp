#pragma once

#include <cstdio>
#include <memory>
#include <bitset>

namespace metronome {

template <typename T, std::size_t N>
class ObjectPool {
  using Storage = typename std::aligned_storage<sizeof(T), alignof(T)>::type;

 public:
  ObjectPool() : storage(new Storage[N]), freeIndices(new std::bitset<N>), size(0) {
    freeIndices->flip();
  }

  ObjectPool(const ObjectPool&) = delete;

  ~ObjectPool() {
    purge();

    delete[] storage;
    delete freeIndices;
  }

  // Create an object in aligned storage
  template <typename... Args>
  T* construct(Args&&... args) {
    if (size >= N) {
      throw std::overflow_error("ObjectPool: Maximum capacity has reached.");
    }

    std::size_t freeIndex = getFreeIndex();
    T* const t = new (storage + freeIndex) T(std::forward<Args>(args)...);
    freeIndices->operator[](freeIndex) = false;
    ++size;

    return t;
  }

  void destruct(T* item) {
    freeIndices->operator[](index(item)) = true;
    --size;
    item->~T();
  }
  
  void purge() {
    for (std::size_t i = 0; i < size; ++i) {
      if (!freeIndices->operator[](i)) {
        reinterpret_cast<const T*>(storage + i)->~T();
        freeIndices->operator[](i) = true;
      }
    }
    
    size = 0;
    lastFreeIndex = 0;
  }

  T* operator[](std::size_t index) const {
    return reinterpret_cast<const T*>(storage + index);
  }

  std::size_t index(const T* item) const {
    return static_cast<std::size_t>(item - reinterpret_cast<T*>(storage));
  }
  
  bool empty() const { return size == 0; }

 private:
  std::size_t getFreeIndex() {
    for (std::size_t i = lastFreeIndex; i < N; ++i) {
      if (freeIndices->operator[](i)) {
        lastFreeIndex = i;
        return i;
      }
    }

    for (std::size_t i = 0; i < N; ++i) {
      if (freeIndices->operator[](i)) {
        lastFreeIndex = i;
        return i;
      }
    }

    throw std::overflow_error("ObjectPool: No free indices are available.");
  }
  
  Storage* storage;
  std::bitset<N>* freeIndices;
  std::size_t size;
  std::size_t lastFreeIndex;
};

}  // namespace metronome
