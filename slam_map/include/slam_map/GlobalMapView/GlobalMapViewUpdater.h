// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <condition_variable>
#include <thread>
#include <Utils/AtomicQueue.h>
#include <slam_map/slam_map.h>
#include <slam_map/slam_mapFwd.h>

/**
 * Class to handle synchronization of GlobalMapView with a slam_map
 *
 * This class will subscribe to a map, receive its callbacks, and, on
 * a separate thread, update the GlobalMapView it's given.
 *
 * It does not own the GlobalMapView* and will not release it when
 * destructed.
 */
class GlobalMapViewUpdater {
 public:
  GlobalMapViewUpdater(const std::shared_ptr<slam_mapProxy>& map,
                       GlobalMapView* global_view);
  virtual ~GlobalMapViewUpdater();
  void Reset(const std::shared_ptr<slam_mapProxy>& map);
  void HandleUpdate(const rslam::map::MapEventUpdate& update);
  void Quit();
  bool IsRunning() const {
    return view_update_running_;
  }

 protected:
  void Run();
  void ViewUpdateLoop();

 private:
  std::shared_ptr<slam_mapProxy> map_;
  GlobalMapView* global_view_;
  std::thread view_update_thread_;
  std::atomic<bool> should_quit_;
  std::atomic<bool> view_update_running_;
  std::shared_ptr<AtomicQueue<rslam::map::MapEventUpdate> > edge_updates_;

  // Describes tree ordering of map from root to leaves
  std::shared_ptr<std::condition_variable> update_cond_;
  std::mutex update_mutex_;
};
