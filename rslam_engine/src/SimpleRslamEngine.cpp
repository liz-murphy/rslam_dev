#include <rslam_engine/SimpleRslamEngine.h>

#include <limits>
#include <rslam_engine/RslamEngine.h>
#include <sparse_front_end/FrontEnd.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/SlamMap.h>
#include <slam_map/MapVisitor/RelativePoseMapVisitor.h>

namespace rslam {

SimpleRslamEngine::SimpleRslamEngine() : engine_(new RslamEngine) {}

SimpleRslamEngine::~SimpleRslamEngine() {}

void SimpleRslamEngine::Init(const RslamEngineOptions& options,
                             const calibu::CameraRigT<Scalar>& rig,
                             const std::shared_ptr<pb::ImageArray>& images) {
  engine_->Reset(options, rig);
  engine_->Init(images);
}

ReferenceFrameId SimpleRslamEngine::AddFrame(
    const std::shared_ptr<pb::ImageArray>& images) {
  engine_->Iterate(images);

  SlamFramePtr curr_frame = engine_->frontend_->current_frame();
  CHECK(curr_frame);
  return curr_frame->id();
}

bool SimpleRslamEngine::CurrentFrameId(ReferenceFrameId* id) const {
  CHECK(id);
  SlamFramePtr curr_frame = engine_->frontend_->current_frame();
  if (!curr_frame) return false;

  *id = curr_frame->id();
  return true;
}

bool SimpleRslamEngine::GetRelativePose(const ReferenceFrameId& other,
                                        Sophus::SE3t* t_ab) const {
  SlamFramePtr curr_frame = engine_->frontend_->current_frame();
  if (!curr_frame) return false;

  return GetRelativePose(curr_frame->id(), other, t_ab);
}

bool SimpleRslamEngine::GetRelativePose(const ReferenceFrameId& a,
                                        const ReferenceFrameId& b,
                                        Sophus::SE3t* t_ab) const {
  RelativePoseMapVisitor visitor(b);
  visitor.set_root_id(a);
  visitor.set_depth(MapVisitor::kMaxDepth);

  engine_->map_->BFS(&visitor);
  return visitor.relative_pose(t_ab);
}

bool SimpleRslamEngine::IsInitialized() const {
  return engine_->frontend_->IsInitialized();
}
}  // namespace rslam
