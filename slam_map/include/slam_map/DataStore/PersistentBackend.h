// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <pthread.h>
#include <slam_map/uuid.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <CVars/CVar.h>
#include <miniglog/logging.h>
#include <Utils/AtomicQueue.h>

#include <rslam.pb.h>

template <typename TableT>
class PersistentBackend {
 public:
  typedef typename TableT::pointer_type pointer_type;
  typedef typename TableT::id_type id_type;
  typedef std::function<void(const pointer_type& pointer)> OnLoadCallback;

  /**
   * Create a new persistent object backend
   *
   * @param filename Name for database
   * @param erase_existing If the file is present, should be erased immediately
   * @param num_db_connections # of persistent database connections.
   */
  explicit PersistentBackend(const std::string& filename,
                             bool erase_existing,
                             int num_db_connections);
  ~PersistentBackend();

  void Flush();
  void Add(const id_type& id, const pointer_type& ptr);
  pointer_type Get(const id_type& id);
  void Set(const id_type& id, const pointer_type& ptr);
  void Clear();

  template <typename IdT>
  void UniqueIds(int column, std::set<IdT>* ids) const;

  /** Ensure that the PersistentBackend is in a consistent state */
  void CheckConsistency();

  uint64_t Size() const;

  /** Does this backend meet the requirements for eviction? */
  bool ShouldEvict() const;

  /** Evict objects which are no longer needed */
  void Evict();

  /** Wait until an object is added to this backend or the time expires */
  void WaitUntilAdd(const std::chrono::milliseconds& time);

  bool IsInMemory(const id_type& id);

  /** Get the minimum suggested batch write size */
  uint64_t batch_write_min() const      { return batch_write_min_;  }
  void set_batch_write_min(uint64_t s)  { batch_write_min_ = s;     }

  /** Get the maximum suggested batch write size */
  uint64_t batch_write_max() const      { return batch_write_max_;  }
  void set_batch_write_max(uint64_t s)  { batch_write_max_ = s;     }

  /** Get the maximum suggested batch write size */
  uint64_t in_mem_goal() const          { return in_mem_goal_;      }
  void set_in_mem_goal(uint64_t s)      { in_mem_goal_ = s;         }

  void set_on_load_callback(const OnLoadCallback& c) {
    on_load_callback_ = c;
  }

 protected:
  void GatherUnused(std::vector<std::pair<id_type, pointer_type> >* to_write);

  /** Should the given Id be evicted? */
  bool ShouldEvict(const id_type& id) const;

  /**
   * Write a vector of frames to disk AND erase them from the map
   */
  void WriteBatchAndErase(
      std::vector<std::pair<id_type, pointer_type> >* to_write);

  pointer_type InsertExisting(const id_type& id, const pointer_type& ptr);

  /** Find the object for a given ID if it exists in memory currently */
  pointer_type Find(const id_type& id);

  /** Get the desired object into memory and return it */
  pointer_type Load(const id_type& id);

  std::shared_ptr<TableT> GetTable() const;

 private:
  mutable AtomicQueue<std::shared_ptr<TableT> > tables_;
  mutable std::unordered_map<id_type, pointer_type> objects_in_use_;
  mutable AtomicQueue<id_type> id_queue_;
  mutable std::mutex add_mutex_;
  std::condition_variable add_signal_;

  const std::string filename_;

  uint64_t in_mem_goal_, batch_write_min_, batch_write_max_;
  int batch_write_per_transaction_;
  int num_connections_;

  uint64_t size_;
  mutable pthread_rwlock_t map_lock_;
  id_type last_id_;

  /** @todo Add a cvar to control this... */
  uint32_t debug_level_;

  OnLoadCallback on_load_callback_;
};

template <typename TableT>
PersistentBackend<TableT>::PersistentBackend(
    const std::string& file, bool erase_existing, int num_db_connections) :
    filename_(file),
    in_mem_goal_(300),
    batch_write_min_(200),
    batch_write_max_(500),
    batch_write_per_transaction_(50),
    num_connections_(num_db_connections),
    size_(0),
    debug_level_(1) {

  if (pthread_rwlock_init(&map_lock_, NULL)) {
    LOG(FATAL) << "Initializing rwlocks in PersistentBackend failed.";
  }

  for (int i = 0; i < num_connections_; ++i) {
    auto table = std::make_shared<TableT>(file);
    table->Init();
    tables_.push(table);
  }

  if (erase_existing) {
    auto t = tables_.front();
    t->Clear();
  } else {
    size_ = tables_.front()->Size();
  }
}

template <typename TableT>
PersistentBackend<TableT>::~PersistentBackend() {
  add_signal_.notify_all();
  Flush();

  if (pthread_rwlock_destroy(&map_lock_)) {
    LOG(FATAL) << "Destroying rwlock in PersistentBackend failed.";
  }

  while (!tables_.empty()) {
    tables_.front()->Close();
    tables_.pop();
  }
}

template <typename TableT>
void PersistentBackend<TableT>::Flush() {
  CHECK_EQ(pthread_rwlock_wrlock(&map_lock_), 0);

  std::vector<std::pair<id_type, pointer_type> > to_write;
  for (auto& it : objects_in_use_) {
    to_write.emplace_back(it.first, it.second);
  }
  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);

  bool success = false;

  auto table = GetTable();
  while (!success) {
    table->BeginTransaction();
    for (const auto& pair : to_write) {
      table->Write(pair.first, pair.second);
    }
    success = table->CommitTransaction();
    LOG_IF(ERROR, !success) << "Flush failed. Retrying.";
  }
  tables_.push(table);
}

template <typename TableT>
void PersistentBackend<TableT>::Add(const id_type& id,
                                    const pointer_type& ptr) {
  CHECK_EQ(pthread_rwlock_wrlock(&map_lock_), 0);

  // Need to skip if we've already added this id
  if (objects_in_use_.count(id) != 0) {
    CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
    return;
  }

  objects_in_use_.insert({id, ptr});

  if (size_ == 0) {
    last_id_ = id;
  } else if (last_id_ < id) {
    last_id_ = id;
  }
  ++size_;

  id_queue_.push(id);
  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);

  add_signal_.notify_all();
}

template <typename TableT>
auto PersistentBackend<TableT>::Get(const id_type& id) -> pointer_type {
  CHECK_EQ(pthread_rwlock_rdlock(&map_lock_), 0);
  auto it = objects_in_use_.find(id);
  if (it != objects_in_use_.end()) {
    pointer_type ptr = it->second;
    CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
    return ptr;
  }
  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
  return Load(id);
}

template <typename TableT>
void PersistentBackend<TableT>::Set(const id_type& id,
                                    const pointer_type& ptr) {
  CHECK_EQ(pthread_rwlock_wrlock(&map_lock_), 0);

  assert(objects_in_use_.count(id));
  objects_in_use_[id] = ptr;

  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
}

/** Find the object for a given ID if it exists in memory currently */
template <typename TableT>
auto PersistentBackend<TableT>::Find(const id_type& id) -> pointer_type {
  CHECK_EQ(pthread_rwlock_rdlock(&map_lock_), 0);
  auto it = objects_in_use_.find(id);
  auto end = objects_in_use_.end();
  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);

  if (it == end) {
    return nullptr;
  }
  return it->second;
}

template <typename TableT>
bool PersistentBackend<TableT>::IsInMemory(const id_type& id) {
  CHECK_EQ(pthread_rwlock_rdlock(&map_lock_), 0);
  size_t count = objects_in_use_.count(id);
  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
  return count > 0;
}

template <typename TableT>
void PersistentBackend<TableT>::Evict() {
  if (!ShouldEvict()) return;

  CheckConsistency();
  std::vector<std::pair<id_type, pointer_type> > to_write;
  GatherUnused(&to_write);

  if (to_write.size() > batch_write_min()) {
    WriteBatchAndErase(&to_write);
  } else {
    for (const std::pair<id_type, pointer_type>& v : to_write) {
      id_queue_.push(v.first);
    }
  }
}

template <typename TableT>
void PersistentBackend<TableT>::Clear() {
  auto table = GetTable();
  table->Clear();
  tables_.push(table);

  CHECK_EQ(pthread_rwlock_wrlock(&map_lock_), 0);
  objects_in_use_.clear();
  size_ = 0;
  id_queue_.clear();

  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
}

template <typename TableT>
template <typename IdT>
void PersistentBackend<TableT>::UniqueIds(int column,
                                          std::set<IdT>* ids) const {
  IdT to_add;
  for (const auto& pair : objects_in_use_) {
    TableT::ConvertId(pair.first, &to_add);
    ids->insert(to_add);
  }

  auto table = GetTable();
  table->UniqueIds(column, ids);
  tables_.push(table);
}

template <typename TableT>
void PersistentBackend<TableT>:: CheckConsistency() {
  CHECK_EQ(pthread_rwlock_rdlock(&map_lock_), 0);
  size_t queue_size = id_queue_.size();
  size_t store_size = objects_in_use_.size();
  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);

  if (queue_size != store_size) {
    LOG(FATAL) << "Queue and store are out of sync: [ "
               << queue_size << ", " << store_size
               << "]. Abort().";
  }
}

/**
 * Write a vector of frames to disk AND erase them from the map
 */
template <typename TableT>
void PersistentBackend<TableT>::WriteBatchAndErase(
    std::vector<std::pair<id_type, pointer_type> >* to_write) {
  std::vector<std::pair<id_type, pointer_type> > erased;
  erased.reserve(to_write->size());

  auto table = GetTable();
  while (!to_write->empty()) {
    erased.clear();  // Clear this when no mutex is held.

    table->BeginTransaction();
    for (int i = 0; i < batch_write_per_transaction_ &&
             !to_write->empty(); ++i) {
      const std::pair<id_type, pointer_type>& v = to_write->back();
      const id_type& id = v.first;
      const pointer_type& ptr = v.second;

      CHECK_EQ(pthread_rwlock_rdlock(&map_lock_), 0);
      auto it = objects_in_use_.find(ptr->id());
      auto end = objects_in_use_.end();
      CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);

      // Make sure this pointer is actually still in memory
      if (it == end) {
        to_write->pop_back();
        continue;
      }

      CHECK_EQ(pthread_rwlock_wrlock(&map_lock_), 0);

      // Use count should be only 2: this function and the main map
      if (ptr && ptr.use_count() == 2) {
        objects_in_use_.erase(id);
        CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);

        table->Write(id, ptr);
        erased.emplace_back(v);
      } else {
        CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
        id_queue_.push(id);
      }
      to_write->pop_back();
    }

    // If the transaction fails, we need to reinsert all the erased pointers
    if (!table->CommitTransaction()) {
      LOG(ERROR) << "WriteBatchAndErase commit failed. Replacing objects";
      CHECK_EQ(pthread_rwlock_wrlock(&map_lock_), 0);
      objects_in_use_.insert(erased.begin(), erased.end());
      for (const auto& pair : erased) {
        id_queue_.push(pair.first);
      }
      CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
    }
  }
  tables_.push(table);
}

template <typename TableT>
std::shared_ptr<TableT> PersistentBackend<TableT>::GetTable() const {
  std::shared_ptr<TableT> table;
  while (!tables_.pop_front(&table)) {}
  return table;
}

template <typename TableT>
auto PersistentBackend<TableT>::Load(const id_type& id) -> pointer_type {
  pointer_type output_ptr(new typename pointer_type::element_type);
  auto table = GetTable();
  bool success = table->Read(id, &output_ptr);
  if (success && on_load_callback_) {
    on_load_callback_(output_ptr);
  }
  tables_.push(table);
  return success ? InsertExisting(id, output_ptr) : nullptr;
}

template <typename TableT>
auto PersistentBackend<TableT>::InsertExisting(
    const id_type& id, const pointer_type& to_insert) -> pointer_type {
  CHECK_EQ(pthread_rwlock_wrlock(&map_lock_), 0);

  auto it = objects_in_use_.find(id);
  // Check again to prevent a data race...
  if (it != objects_in_use_.end()) {
    pointer_type ptr = it->second;
    CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
    return ptr;
  }

  objects_in_use_[id] = to_insert;
  id_queue_.push(id);

  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);
  return to_insert;
}

template <typename TableT>
void PersistentBackend<TableT>::GatherUnused(
    std::vector<std::pair<id_type, pointer_type> >* to_write) {
  size_t queue_size = id_queue_.size();

  for (size_t i = 0; i < queue_size; ++i) {
    // Look for obj to evict
    id_type id = id_queue_.front();

    // Only evict if it's far enough behind us
    if (!ShouldEvict(id)) {
      id_queue_.requeue_front();
      continue;
    }

    pointer_type ptr = Find(id);
    if (!ptr) {
      LOG(WARNING) << "Queued ID (" << id
                   << ")does not exist in map anymore.";
      id_queue_.pop();
      continue;
    }

    // Check to see if this is the only remaining copy in memory
    if (ptr && ptr.use_count() == 2) {
      to_write->emplace_back(id, ptr);
      id_queue_.pop();
      if (to_write->size() > batch_write_min()) {
        return;
      }
    } else {
      id_queue_.requeue_front();
    }
  }
}

template <typename TableT>
bool PersistentBackend<TableT>::ShouldEvict(const id_type& id) const {
  CHECK_EQ(pthread_rwlock_rdlock(&map_lock_), 0);
  id_type last = last_id_;
  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);

  return last.session_id != id.session_id || (last.id - id.id) > 50;
}

template <typename TableT>
bool PersistentBackend<TableT>::ShouldEvict() const {
  return id_queue_.size() > in_mem_goal_ + batch_write_max_;
}

template <typename TableT>
void PersistentBackend<TableT>::WaitUntilAdd(
    const std::chrono::milliseconds& time) {
  std::unique_lock<std::mutex> lock(add_mutex_);
  add_signal_.wait_for(lock, time);
}

template <typename TableT>
uint64_t PersistentBackend<TableT>::Size() const {
  CHECK_EQ(pthread_rwlock_rdlock(&map_lock_), 0);
  size_t map_size = size_;
  CHECK_EQ(pthread_rwlock_unlock(&map_lock_), 0);

  return map_size;
}
