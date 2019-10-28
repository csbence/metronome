#pragma once

#include <cstddef>
#include <vector>

namespace metronome {

template <typename T, std::size_t BUCKET_COUNT>
class BucketPriorityQueue {
  class Bucket {

    void push(T&& item) {
      items.emplace_back(std::move(item));
      // TODO set secondary index
      // item.bucketIndex = items.size() - 1;
    }

    void push(T& item) {
      items.emplace_back(std::move(item));
    }

    void remove(T item) {
      //TODO swap the item with the last element and then remove it
      // Make sure the item's index is reset and the swapped items index is updated
    }

    T pop() {
      // TODO throw if items is empty
      T item = std::move(items.back());
      items.pop_back();

      return item;
    }

    std::vector<T> items;
  };

  class BucketAlignment {
   public:
    BucketAlignment(const size_t fBucketCount,
                    const size_t dBucketCount,
                    const double fLowerBound,
                    const double fUpperBound)
        : fBucketCount(fBucketCount),
          dBucketCount(dBucketCount),
          fLowerBound(fLowerBound),
          fUpperBound(fUpperBound),
          fBucketInterval((fUpperBound - fLowerBound) / fBucketCount) {}

    /** Hash function */
    std::size_t operator()(const T& item) const {
      // TODO Check if f is smaller than the lower bound and throw
      // TODO Check id d is smaller than the upper bound

      const double fOffset = item.f - fLowerBound;
      const std::size_t fIndex = fOffset % fBucketInterval;
      const std::size_t dIndex = item.d;
      const std::size_t index = fIndex * dBucketCount + dIndex;

      return index;
    }

    /** Equals function */
    std::size_t operator()(const T& lhs, const T& rhs) const {
      // Two items belong to the same bucket iff their index is the same
      // as we have a perfect hash function
      return operator()(lhs) == operator()(rhs);
    }

    /** Determines if the element is in the active set. */
    bool isActive(const T& item) const { return item.f < fUpperBound; }

    std::size_t getInactiveIndex() const { return fBucketCount * dBucketCount; }

    const std::size_t fBucketCount;
    const std::size_t dBucketCount;
    const double fLowerBound;
    const double fUpperBound;
    const double fBucketInterval;
  };

  BucketPriorityQueue(BucketAlignment bucketAlignment)
      : bucketAlignment(bucketAlignment) {

    // Allocate empty buckets
    buckets.resize(bucketAlignment.getInactiveIndex());
  }

  void push(T item) {
    // TODO check index and throw if not "infinity"

    // Item is inactive
    if (not bucketAlignment.isActive(item)) {
      item.index = bucketAlignment.getInactiveIndex();
      inactiveItems.push_back(std::move(item));
      // We might want to do some maintenance on the inactive items
      // However, in that case it we should probably have a separate container
      // for the inactive elements
      return;
    }

    std::size_t itemIndex = bucketAlignment(item);
    buckets[itemIndex].push(item);
  }

//  void checkRebucket() {
//    // turn off rebucket for debugging
//    // rebucket when the last bucket has more than 50% of the items
//    if (numberOfBuckets == 1) {
//      return;
//    }
//    auto lCompare = (double)bucketSizes[numberOfBuckets - 1];
//    auto rCompare = (double)size / 2.0;
//    if (lCompare >= rCompare) {
//      rebucket();
//    }
//  }

  T pop() {
    // TODO check and throw if empty

    // TODO:
    // 1. get current bucket

    auto bucket = currentBucket();
    auto node = queue[bucket].pop();

    // set the index
    node->index = undefined;

    // update the size
    --size;
    --bucketSizes[bucket];

    return node;
  }

  void insertOrUpdate(T& item) {
    if (item.index == undefined) {
      // item is not yet in the queue
      push(item);
    } else {
      // already in the queue lets update it
      update(item);
    }
  }

  void update(T& item) {
    // If belongs to the same bucket don't do anything
    const std::size_t newIndex = bucketAlignment(item);
    if (newIndex == item.index) return;

    // We probably need a secondary index to do this efficiently
    buckets[item.index].remove(item);
    buckets[newIndex].push(item);


    // remove the item from old location
    // decide its location and push it
    queue[item.index].erase(item.pos, item.innerIndex);
    auto newIndex = decideInterval(item);
    queue[newIndex].push(item);
    item.index = newIndex;
  }

  [[nodiscard]] bool isEmpty() const { return size == 0; }

  void setMin(double value) { currentMin = value; }

  void setMax(double value) { currentMax = value; }

  // recalculate the f intervals of buckets
  // each bucket holds nodes within that interval
  void setIntervals() {
    const auto minMaxDiff = currentMax - currentMin;
    const auto bucketInterval = minMaxDiff / (numberOfBuckets - 1);
    auto currentBucketMin = currentMin;
    for (int i = 0; i < numberOfBuckets; ++i) {
      auto lower = currentBucketMin;
      auto upper = currentBucketMin + bucketInterval;
      intervals[i].first = lower;
      intervals[i].second = upper;
      currentBucketMin += bucketInterval;
    }
  }

 private:
  // decide where a new node goes based on its f value
  std::size_t decideInterval(const T& item) {
    if (numberOfBuckets == 1) {
      return 0;
    }
    if (currentMin == currentMax && item.f == currentMin) {
      return 0;
    }
    if (item.f > currentMax) {
      return numberOfBuckets - 1;
    }
    for (int i = 0; i < numberOfBuckets; ++i) {
      if (item.f >= intervals[i].first && item.f <= intervals[i].second) {
        return i;
      }
    }
    return numberOfBuckets - 1;
  }

  // method for debugging implementation eventually remove
  LinkedList<T>* nextNode() {
    for (int i = 0; i < numberOfBuckets; ++i) {
      if (!queue[i].empty()) {
        for (int j = 0; j < max; ++j) {
          if (!queue[i][j].empty()) {
            return &queue[i][j];
          }
        }
      }
    }
    return nullptr;
  }

  void rebucket() {
    for (auto& bucket : queue) {
      std::cout << bucket << std::endl;
    }
    std::cout << "rebucket" << std::endl;
    auto bucket = currentBucket();
    currentMin = intervals[bucket].first;  // lower bound on the currentBucket
    currentMax = nextMax;  // the next max value found while inserting
    setIntervals();        // recalculate the intervals
    // create the new queue
    std::vector<BucketList<T>> newQueue;
    newQueue.reserve(numberOfBuckets);
    for (int i = 0; i < numberOfBuckets; ++i) {
      newQueue.emplace_back(max);
    }
    std::vector<int> newBucketSizes(numberOfBuckets);
    while (!this->isEmpty()) {
      // remove the node
      std::cout << "\tremoving " << size << std::endl;
      auto node = pop();

      // decide the new bucket
      auto interval = decideInterval(*node);
      ++newBucketSizes[interval];

      // insert the node
      newQueue[interval].push(*node);

      node->index = interval;
    }
    queue = std::move(newQueue);
    bucketSizes = std::move(newBucketSizes);
    size = bucketSizes[0];  // we always rebucket everything to the first bucket
  }

  // first non-empty bucket
  int currentBucket() {
    for (int i = 0; i < numberOfBuckets; ++i) {
      if (bucketSizes[i] > 0) {
        return i;
      }
    }
    return numberOfBuckets - 1;
  }

  std::vector<BucketList<T>> queue;

  std::size_t size;
  double currentMin;
  double currentMax;
  double nextMax;
  std::size_t numberOfBuckets;
  std::size_t max;

  std::vector<std::pair<double, double>> intervals;
  std::vector<int> bucketSizes;

  BucketAlignment bucketAlignment;
  std::vector<Bucket> buckets;
  std::vector<T> inactiveItems;
};

}