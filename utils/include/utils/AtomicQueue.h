// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <pthread.h>
#include <queue>
#include <mutex>

#include <boost/lockfree/queue.hpp>

template <typename T>
class AtomicQueue : public boost::lockfree::queue<T>
{
};
/**
 * Adds basic thread-safe read/write wrapper around a queue
 *
 * Additionally, adds clear() and requeue_front().
 */
/*template <typename T, typename QueueT = std::queue<T> >
class AtomicQueue {
 public:
  AtomicQueue() {
    int result = pthread_rwlock_init(&lock_, NULL);
    CHECK_EQ(0, result);
  }

  ~AtomicQueue() {
    int result = pthread_rwlock_destroy(&lock_);
    CHECK_EQ(0, result);
  }

  void pop() {
    CHECK_EQ(0, pthread_rwlock_wrlock(&lock_));
    queue_.pop();
    CHECK_EQ(0, pthread_rwlock_unlock(&lock_));
  }

  void push(const T& v) {
    CHECK_EQ(0, pthread_rwlock_wrlock(&lock_));
    queue_.push(v);
    CHECK_EQ(0, pthread_rwlock_unlock(&lock_));
  }

  T front() const {
    CHECK_EQ(0, pthread_rwlock_rdlock(&lock_));
    T front_val = queue_.front();
    CHECK_EQ(0, pthread_rwlock_unlock(&lock_));

    return front_val;
  }

  /**
   * Get first value and pop from queue.
   * Returns false if the queue is empty.
   */
 /* bool pop_front(T* out) {
    CHECK_EQ(0, pthread_rwlock_wrlock(&lock_));
    bool empty = queue_.empty();
    if (!empty) {
      *out = queue_.front();
      queue_.pop();
    }
    CHECK_EQ(0, pthread_rwlock_unlock(&lock_));

    return !empty;
  }

  /** Take object at head and push to the tail */
  /*void requeue_front() {
    CHECK_EQ(0, pthread_rwlock_wrlock(&lock_));
    queue_.push(queue_.front());
    queue_.pop();
    CHECK_EQ(0, pthread_rwlock_unlock(&lock_));
  }

  size_t size() const {
    CHECK_EQ(0, pthread_rwlock_rdlock(&lock_));
    size_t s = queue_.size();
    CHECK_EQ(0, pthread_rwlock_unlock(&lock_));

    return s;
  }

  bool empty() const {
    CHECK_EQ(0, pthread_rwlock_rdlock(&lock_));
    bool e = queue_.empty();
    CHECK_EQ(0, pthread_rwlock_unlock(&lock_));

    return e;
  }

  void clear() {
    CHECK_EQ(0, pthread_rwlock_wrlock(&lock_));
    queue_ = QueueT();
    CHECK_EQ(0, pthread_rwlock_unlock(&lock_));
  }

 private:
  mutable pthread_rwlock_t lock_;
  QueueT queue_;
};*/
