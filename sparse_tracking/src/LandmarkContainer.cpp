#include <sparse_tracking/LandmarkContainer.h>
#include <slam_map/SlamMap.h>
#include <sparse_tracking/LocalMap.h>
#include <slam_map/TransformEdgeId.h>

LandmarkContainer::LandmarkContainer() : state(eLmkActive),
                                         x_r(Eigen::Vector4t::Zero()),
                                         x_w(Eigen::Vector4t::Zero()),
                                         ba_id(-1),
                                         depth_ratio(0),
                                         base_camera_id(-1),
                                         num_frames_visible(-1) {
}

LandmarkContainer::LandmarkContainer(const Landmark& lmk,
                                     const SlamMap* slam_map) :
    state(lmk.state()),
    id(lmk.id()),
    x_r(lmk.xrp()),
    x_w(Eigen::Vector4t::Zero()),
    ba_id(UINT_MAX),
    depth_ratio(1.0),
    base_camera_id(lmk.base_camera()),
    num_frames_visible(0),
    orientation_3d(lmk.orientation()) {
  MultiViewMeasurement z;
  for (const MeasurementId& z_id : lmk.GetFeatureTrackRef()) {
    bool good_z = false;
    if (!slam_map->GetMeasurement(z_id, z)) continue;
    for (size_t ii = 0 ; ii < z.NumCameras() ; ii++) {
      if (z.HasGoodMeasurementInCam(ii)) {
        measurements.emplace_back(new MeasurementContainer(z, ii));
        good_z = true;
      }
    }
    if (good_z) {
      num_frames_visible++;
    }
  }
}
