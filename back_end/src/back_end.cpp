#include <back_end/back_end.h>

#include <slam_map/SlamMap.h>
#include <slam_map/TransformEdge.h>

namespace rslam {
namespace backend {

static int min_frames_between_pgr = 100;
/*CVarUtils::CreateCVar<int>(
        "backend.FramesBetweenPGR", 100,
        "Minimum number of frames between Pose Graph Relaxations");*/

static int pgr_depth = 1000;
   /* CVarUtils::CreateCVar<int>(
        "backend.PGRDepth", 1000, "Depth of Pose Graph Relaxation.");*/

BackEnd::BackEnd() : is_running_(false) {}

BackEnd::~BackEnd() {}

void BackEnd::Init(const std::shared_ptr<SlamMap>& map) {
  std::lock_guard<std::mutex> lock(pgr_mutex_);
  map_ = map;
  pgr_optimization_.Init(map);
}

void BackEnd::Run() {
  CHECK(map_) << "BackEnd requires Init() with a valid map before Run()";

  is_running_ = true;
  pgr_thread_ = std::thread(&BackEnd::PGRThread, this);
}

static inline void WaitThread(std::thread* t) {
  if (t->joinable()) {
    t->join();
  }
}

void BackEnd::Stop() {
  is_running_ = false;
  WaitThread(&pgr_thread_);
}

void BackEnd::PGRThread() {
  static const std::chrono::milliseconds kWaitTime(500);
  // Listen for map updates such as PoseMeasurementAdded or EdgeAdded

  std::atomic<int> num_edges_since_pgr(0);
  std::condition_variable pgr_cond;
  std::mutex pgr_thread_mutex;
  map::MapEventUpdate pgr_update;

  map_->Subscribe({map::kAddEdgeMapEvent},
                  [&](const map::MapEventUpdate& u) {
                    if (u.event != map::kAddEdgeMapEvent) return;
                    if (++num_edges_since_pgr > min_frames_between_pgr &&
                        pgr_mutex_.try_lock()) {
                      pgr_update = u;
                      pgr_mutex_.unlock();
                      pgr_cond.notify_one();
                    }
                  });

  while (is_running_) {
    std::unique_lock<std::mutex> lock(pgr_thread_mutex);
    if (pgr_cond.wait_for(lock, kWaitTime) == std::cv_status::timeout) {
      continue;
    }
    CHECK_EQ(map::kAddEdgeMapEvent, pgr_update.event);
    bool loop_closed = false;
    if (SlamEdgePtr edge = map_->GetEdgePtr(pgr_update.payload.edge)) {
      loop_closed = edge->is_loop_closure();
    }

    if (loop_closed) {
      PoseGraphRelaxation(pgr_update.payload.edge.start);
      num_edges_since_pgr.store(0);
    }
  }
}

void BackEnd::AACBundleAdjustment(const ReferenceFrameId& root) {

}

void BackEnd::PoseGraphRelaxation(const ReferenceFrameId& root) {
  std::lock_guard<std::mutex> lock(pgr_mutex_);

  /** @todo Intelligently decide how deep to do the PGR */
  pgr_optimization_.RelaxMap(pgr_depth, root, 10);
}
}  // end namespace backend
}  // end namespace rslam
