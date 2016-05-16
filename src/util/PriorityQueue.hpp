#ifndef RITMUS_PRIORITY_QUEUE_HPP
#define RITMUS_PRIORITY_QUEUE_HPP

#include <boost/assert.hpp>
#include <functional>
#include <vector>

template <typename T> class PriorityQueue {
public:
  PriorityQueue(unsigned int capacity,
                std::function<int(const T &, const T &)> comparator)
      : capacity(capacity), comparator(comparator), queue(capacity), size(0) {}

  void push(const T &item) {
    if (size == capacity) {
      throw std::overflow_error(
          "Priority queue reached its maximum capacity: " + capacity);
    }

    if (size == 0) {
      queue[0] = item;
      item.index = 0;
    } else {
      siftUp(size, item);
    }

    ++size;
  }

  T &pop() {
    if (size == 0) {
      return nullptr;
    }

    --size;
    auto result = queue[0];
    auto x = queue[size];
    queue[size] = nullptr;

    if (size != 0) {
      siftDown(0, x);
    }

    BOOST_ASSERT_MSG(result.index == 0,
                     "Internal index of top item was not null");
    return result;
  }

  const T &top() const {
    if (size == 0) {
      return nullptr;
    }

    return queue[0];
  }

  unsigned int getCapacity() const { return capacity; }

private:
  void siftUp(const unsigned int index, const T &item) {
    unsigned int currentIndex = index;
    while (currentIndex > 0) {
      const auto parentIndex = (currentIndex - 1) >> 1;
      const auto parentItem = queue[parentIndex];

      if (comparator(item, parentItem) >= 0) {
        break;
      }

      // Move parent down and update its index
      queue[currentIndex] = parentItem;
      parentItem.index = currentIndex;
      currentIndex = parentIndex;
    }

    queue[currentIndex] = item;
    item.index = currentIndex;
  }

  void siftDown(const unsigned int index, const T &item) {
    auto currentIndex = index;
    const unsigned int half = size >> 1;

    while (currentIndex < half) {
      auto childIndex = (currentIndex << 1) + 1;
      auto childItem = queue[childIndex];
      auto rightIndex = childIndex + 1;

      if (rightIndex < size && comparator(childItem, queue[rightIndex]) > 0) {
        childIndex = rightIndex;
        childItem = queue[rightIndex];
      }

      if (comparator(item, childItem) <= 0) {
        break;
      }

      queue[currentIndex] = childItem;
      childItem.index = currentIndex;
      currentIndex = childIndex;
    }

    queue[currentIndex] = item;
    item.index = currentIndex;
  }

  const unsigned int capacity;
  std::function<int(const T &, const T &)> comparator;
  std::vector<T> queue;
  unsigned int size;
};

#endif // RITMUS_PRIORITY_QUEUE_HPP
