#pragma once

#include <cassert>
#include <cstddef>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace cserna {

template <typename T, typename Hash = std::hash<T>, typename Equal = std::equal_to<T>>
class NonIntrusiveIndexFunction {
 public:
  std::size_t& operator()(T& item) {
      const auto itemIterator = indexMap.find(item);

      if (itemIterator == indexMap.cend()) {
          std::size_t& index = indexMap[item];
          index = std::numeric_limits<std::size_t>::max();
          return index;
      } else {
          return itemIterator->second;
      }
  }

  std::size_t operator()(const T& item) const {
      const auto itemIterator = indexMap.find(item);
      if (itemIterator == indexMap.cend()) {
          return std::numeric_limits<std::size_t>::max();
      } else {
          return itemIterator->second;
      }
  }

 private:
  std::unordered_map<T, std::size_t, Hash, Equal> indexMap;
};

template <typename T, typename Comparator = std::less<T>>
class ThreeWayComparatorAdapter {
 public:
  int operator()(const T& lhs, const T& rhs) const {
      if (comparator(lhs, rhs))
          return -1;
      if (comparator(rhs, lhs))
          return 1;
      return 0;
  }

 private:
  const Comparator comparator;
};

template <typename T,
          typename IndexFunction,
          typename ThreeWayComparator,
    std::size_t INITIAL_CAPACITY = 0,
    std::size_t MAX_CAPACITY = std::numeric_limits<std::size_t>::max()>
class DynamicPriorityQueue {
 public:
  explicit DynamicPriorityQueue(ThreeWayComparator comparator = ThreeWayComparator(),
                                const IndexFunction indexFunction = IndexFunction())
      : comparator{std::move(comparator)}, indexFunction{std::move(indexFunction)}, queue{} {
      queue.reserve(INITIAL_CAPACITY);
  }

  DynamicPriorityQueue(const DynamicPriorityQueue&) = delete;
  DynamicPriorityQueue(DynamicPriorityQueue&&) noexcept = default;
  ~DynamicPriorityQueue() = default;

  void push(T item) {
      if (queue.size() == MAX_CAPACITY) {
          throw std::overflow_error("Priority queue reached its maximum capacity:" + std::to_string(MAX_CAPACITY));
      }

      if (queue.size() == 0) {
          indexFunction(item) = 0;
          queue.push_back(item);
      } else {
          queue.push_back(item);
          siftUp(queue.size() - 1, std::move(item));
      }
  }

  T pop() {
      if (queue.size() == 0) {
          throw std::underflow_error("Priority queue is empty.");
      }

      auto top_item(std::move(queue[0]));

      if (queue.size() == 1) {
          indexFunction(top_item) = std::numeric_limits<std::size_t>::max();
          queue.pop_back();
          return top_item;
      }

      auto last_item(std::move(queue[queue.size() - 1]));

      queue.pop_back();
      siftDown(0, std::move(last_item));

      assert(indexFunction(top_item) == 0 &&
          "Internal index of top item was "
          "non-zero");

      indexFunction(top_item) = std::numeric_limits<std::size_t>::max();
      return top_item;
  }

  T& top() {
      if (queue.size() == 0) {
          throw std::underflow_error("Priority queue is empty.");
      }

      return queue[0];
  }

  const T& top() const {
      if (queue.size() == 0) {
          throw std::underflow_error("Priority queue is empty.");
      }

      return queue[0];
  }

  void remove(T item) {
      if (!contains(item)) {
          return;
      }

      const auto index = indexFunction(item);
      // Invalidate the item's index.
      indexFunction(item) = std::numeric_limits<std::size_t>::max();

      if (index == queue.size() - 1) {
          queue.pop_back();
          return;
      }

      // Override the removed item's slot with the last item
      queue[index] = std::move(queue[queue.size() - 1]);
      indexFunction(queue[index]) = index;

      queue.pop_back();

      // The heap property might've been violated by the swap. Let's fix it.
      siftDown(index, std::move(queue[index]));
  }

  void clear() {
      for (std::size_t i = 0; i < queue.size(); i++) {
          indexFunction(queue[i]) = std::numeric_limits<std::size_t>::max();
      }
      queue.clear();
  }

  void insertOrUpdate(T item) {
      if (indexFunction(item) == std::numeric_limits<std::size_t>::max()) {
          // Item is not in the queue yet
          push(std::move(item));
      } else {
          // Already in the queue
          update(std::move(item));
      }
  }

  void update(T item) {
      const std::size_t originalIndex = indexFunction(item);
      assert(originalIndex != std::numeric_limits<std::size_t>::max() &&
          "Cannot update a node that is not in the queue!");

      const bool moved = siftUp(originalIndex, std::move(item));

      if (!moved) {
          siftDown(originalIndex, std::move(queue[originalIndex]));
      }
  }

  template <typename Action>
  void forEach(Action& action) {
      for (auto& item : queue) {
          action(item);
      }
  }

  std::size_t size() const { return queue.size(); }

  bool empty() const { return queue.size() == 0; }

  bool contains(const T& item) const { return indexFunction(item) != std::numeric_limits<std::size_t>::max(); }

 private:
  bool siftUp(const std::size_t index, T item) {
      auto currentIndex = index;
      while (currentIndex > 0) {
          const auto parentIndex = (currentIndex - 1) / 2;
          auto parentItem = queue[parentIndex];

          if (comparator(item, parentItem) >= 0) {
              break;
          }

          // Move parent down and update its index
          queue[currentIndex] = parentItem;
          indexFunction(parentItem) = currentIndex;
          currentIndex = parentIndex;
      }

      indexFunction(item) = currentIndex;
      queue[currentIndex] = std::move(item);

      return currentIndex == index;
  }

  bool siftDown(const std::size_t index, T item) {
      std::size_t currentIndex = index;
      const std::size_t half = queue.size() / 2;

      while (currentIndex < half) {
          std::size_t childIndex = (currentIndex * 2) + 1;
          T& childItem = queue[childIndex];
          std::size_t rightIndex = childIndex + 1;

          if (rightIndex < queue.size() && comparator(childItem, queue[rightIndex]) > 0) {
              childIndex = rightIndex;
              childItem = queue[rightIndex];
          }

          if (comparator(item, childItem) <= 0) {
              break;
          }

          queue[currentIndex] = childItem;
          indexFunction(childItem) = currentIndex;
          currentIndex = childIndex;
      }

      indexFunction(item) = currentIndex;
      queue[currentIndex] = std::move(item);

      return currentIndex == index;
  }

  const ThreeWayComparator comparator;
  IndexFunction indexFunction;
  std::vector<T> queue;
};

} // namespace cserna
