// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#include <slam_map/GlobalMapView/GlobalMapViewUpdater.h>
#include <slam_map/GlobalMapView/GlobalMapView.h>
#include <slam_map/SlamMap.h>
#include <slam_map/SlamMapProxy.h>

GlobalMapViewUpdater::GlobalMapViewUpdater(
    const std::shared_ptr<SlamMapProxy>& map,
    GlobalMapView* global_view)
    : map_(map), global_view_(global_view), should_quit_(false),
      view_update_running_(false) {
  edge_updates_ = std::make_shared<AtomicQueue<rslam::map::MapEventUpdate> >();
  update_cond_ = std::make_shared<std::condition_variable>();
  Run();
}

GlobalMapViewUpdater::~GlobalMapViewUpdater() {
  Quit();
}

void GlobalMapViewUpdater::Reset(const std::shared_ptr<SlamMapProxy>& map) {
  Quit();
  map_ = map;
  edge_updates_ = std::make_shared<AtomicQueue<rslam::map::MapEventUpdate> >();
  update_cond_ = std::make_shared<std::condition_variable>();
  should_quit_ = false;
  Run();
}

void GlobalMapViewUpdater::ViewUpdateLoop() {
  view_update_running_ = true;
  std::unique_lock<std::mutex> lock(update_mutex_);
  rslam::map::MapEventUpdate update;
  while (!should_quit_) {
    update_cond_->wait(lock);
    while (!should_quit_ && edge_updates_->pop_front(&update)) {
      HandleUpdate(update);
    }
  }
  view_update_running_ = false;
}

void GlobalMapViewUpdater::HandleUpdate(
    const rslam::map::MapEventUpdate& update) {
  if (update.event == rslam::map::kAddEdgeMapEvent) {
    global_view_->AddEdge(update.payload.edge);

  } else if (update.event == rslam::map::kUpdateEdgeMapEvent) {
    global_view_->UpdateEdge(update.payload.edge);

  } else if (update.event == rslam::map::kFinalMapEvent) {
    should_quit_ = true;
  }
}

void GlobalMapViewUpdater::Run() {
  view_update_thread_ = std::thread(
      std::bind(&GlobalMapViewUpdater::ViewUpdateLoop, this));
  static const std::vector<rslam::map::MapEvent> subscriptions =
      {rslam::map::kAddEdgeMapEvent, rslam::map::kUpdateEdgeMapEvent};

  // NOTE: This callback maintains the lifetime of its own state
  // (queue and cond_var) so that we may point this updater at a new
  // map w/o having to ensure the old map has deregistered our
  // callback. The old callback will be called and will do
  // nothing. This could cause problems with an ever growing queue,
  // but we'll cross that bridge when we come to it.
  auto callback = [](
      std::shared_ptr<AtomicQueue<rslam::map::MapEventUpdate> > queue,
      std::shared_ptr<std::condition_variable> update_cond,
      const rslam::map::MapEventUpdate& u) {
    queue->push(u);
    update_cond->notify_all();
  };

  map_->Subscribe(subscriptions, std::bind(
      callback, edge_updates_, update_cond_, std::placeholders::_1));
}

void GlobalMapViewUpdater::Quit() {
  should_quit_ = true;
  update_cond_->notify_all();
  if (view_update_thread_.joinable()) {
    view_update_thread_.join();
  }
}
