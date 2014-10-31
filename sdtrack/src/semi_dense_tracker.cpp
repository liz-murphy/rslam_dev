#include <miniglog/logging.h>
#include <sdtrack/semi_dense_tracker.h>
#define MIN_OBS_FOR_CAM_LOCALIZATION 3
using namespace sdtrack;


void SemiDenseTracker::Initialize(const KeypointOptions &keypoint_options,
                                  const TrackerOptions &tracker_options,
                                  calibu::Rig<Scalar> *rig)
{
  // rig_ = rig;
  // const calibu::CameraModelGeneric<Scalar>& cam = rig->cameras[0].camera;
  // camera_rig_->AddCamera(calibu::CreateFromOldCamera<Scalar>(cam),
  //                       rig->cameras[0].T_wc);
  t_ba_ = Sophus::SE3d();
  camera_rig_ = rig;
  num_cameras_ = camera_rig_->cameras_.size();
  image_pyramid_.resize(num_cameras_);

  keypoint_options_ = keypoint_options;
  tracker_options_ = tracker_options;
  next_track_id_ = 0;
  switch (tracker_options_.detector_type)
  {
    case TrackerOptions::Detector_FAST:
      detector_ = new cv::FastFeatureDetector(
            keypoint_options.fast_threshold,
            keypoint_options.fast_nonmax_suppression);
      break;
    case TrackerOptions::Detector_GFTT:
      detector_ = new cv::GoodFeaturesToTrackDetector(
            keypoint_options.max_num_features /
            powi(tracker_options.feature_cells, 2),
            keypoint_options.gftt_absolute_strength_threshold,
            keypoint_options.gftt_min_distance_between_features,
            keypoint_options.gftt_feature_block_size,
            keypoint_options.gftt_use_harris);
      break;

    case TrackerOptions::Detector_SURF:
      detector_ = new cv::SurfFeatureDetector(1000);
      break;
  }

  uint32_t patch_dim = tracker_options_.patch_dim;
  double robust_norm_thresh = tracker_options_.robust_norm_threshold_;
  pyramid_patch_dims_.resize(tracker_options_.pyramid_levels);
  pyramid_patch_corner_dims_.resize(pyramid_patch_dims_.size());
  pyramid_patch_interp_factors_.resize(pyramid_patch_dims_.size());
  pyramid_error_thresholds_.resize(pyramid_patch_dims_.size());
  // Calculate the pyramid dimensions for each patch. This is used to initialize
  // new patches.
  for (uint32_t ii = 0 ; ii < tracker_options_.pyramid_levels ; ++ii) {
    std::cerr << "Level " << ii << " patch dim is " << patch_dim << std::endl;
    pyramid_error_thresholds_[ii] = robust_norm_thresh;
    robust_norm_thresh *= 2;
    pyramid_patch_dims_[ii] = patch_dim;
    // The array coordinates of the four corners of the patch.
    pyramid_patch_corner_dims_[ii] = {0,                                  // tl
                                      patch_dim - 1,                      // tr
                                      patch_dim * patch_dim - patch_dim,  // bl
                                      patch_dim * patch_dim - 1};         // br

    // For each cell, we also need to get the interpolation factors.
    pyramid_patch_interp_factors_[ii].reserve(powi(patch_dim, 2));
    const double factor = powi(patch_dim - 1, 2);
    for (double yy = 0; yy < patch_dim ; ++yy) {
      for (double xx = 0; xx < patch_dim ; ++xx) {
        pyramid_patch_interp_factors_[ii].push_back({
          ( (patch_dim - 1 - xx) * (patch_dim - 1 - yy) ) / factor,   // tl
          ( xx * (patch_dim - 1 - yy) ) / factor,   // tr
          ( (patch_dim - 1 - xx) * yy ) / factor,   // bl
          ( xx * yy ) / factor   // br
        });
      }
    }
    // patch_dim = (patch_dim + 1) / 2;
  }

  for (int ii = 0; ii < 6 ; ++ii) {
    generators_[ii] = Sophus::SE3d::generator(ii);
  }

  // Inititalize the feature cells.
  feature_cells_.resize(tracker_options_.feature_cells,
                        tracker_options_.feature_cells);
  feature_cells_.setZero();
  feature_cell_rho_.resize(tracker_options_.feature_cells,
                           tracker_options_.feature_cells);
  feature_cell_rho_.setZero();

  mask_.AddImage(rig->cameras_[0]->Width(),
      rig->cameras_[0]->Height());

  pyramid_coord_ratio_.resize(tracker_options_.pyramid_levels);
  current_tracks_.clear();
  new_tracks_.clear();
  previous_keypoints_.clear();
}

void SemiDenseTracker::ExtractKeypoints(const cv::Mat &image,
                                        std::vector<cv::KeyPoint> &keypoints)
{
  std::vector<cv::KeyPoint> cell_kp;
  keypoints.clear();
  keypoints.reserve(keypoint_options_.max_num_features);
  uint32_t cell_width = image.cols / tracker_options_.feature_cells;
  uint32_t cell_height = image.rows / tracker_options_.feature_cells;
  uint32_t cells_hit = 0;
  const double time = Tic();
  for (uint32_t ii = 0  ; ii < tracker_options_.feature_cells ; ++ii) {
    for (uint32_t jj = 0  ; jj < tracker_options_.feature_cells ; ++jj) {
      if (feature_cells_(jj, ii) >= lm_per_cell_) {
        continue;
      }

      const cv::Rect bounds(ii * cell_width, jj * cell_height,
                            cell_width, cell_height);
      cv::Mat roi(image, bounds);
      detector_->detect(roi, cell_kp);

      cells_hit++;

      //      std::cerr << "Detected " << cell_kp.size() << " in " << bounds.x <<
      //                   ", " << bounds.y << ", " << bounds.width << ", " <<
      //                   bounds.height << std::endl;

      // Shift the keypoints.
      for (cv::KeyPoint& kp : cell_kp) {
        kp.pt.x += bounds.x;
        kp.pt.y += bounds.y;
        keypoints.push_back(kp);
      }
    }
  }

  if (tracker_options_.do_corner_subpixel_refinement) {
    std::vector<cv::Point2f> subpixel_centers(keypoints.size());
    for (size_t ii = 0 ; ii < keypoints.size() ; ++ii) {
      subpixel_centers[ii] = keypoints[ii].pt;
    }
    cv::TermCriteria criteria(cv::TermCriteria::COUNT, 10, 0);
    cv::cornerSubPix(image, subpixel_centers,
                     cv::Size(tracker_options_.patch_dim,
                              tracker_options_.patch_dim), cv::Size(-1, -1),
                     criteria);
    for (size_t ii = 0 ; ii < keypoints.size() ; ++ii) {
      // std::cerr << "kp " << ii << " refined from " << keypoints[ii].pt.x << ", "
      //           << keypoints[ii].pt.y << " to " << subpixel_centers[ii].x <<
      //              ", " << subpixel_centers[ii].y << std::endl;
      keypoints[ii].pt = subpixel_centers[ii];
    }
  }

  // std::cerr << "total " << keypoints.size() << " keypoints " << std::endl;
  //  detector_->detect(image, keypoints);
  HarrisScore(image.data, image.cols, image.rows,
            tracker_options_.patch_dim, keypoints);
  std::cerr << "extract feature detection for " << keypoints.size() <<
               " and "  << cells_hit << " cells " <<  " keypoints took " <<
               Toc(time) << " seconds." << std::endl;
}

bool SemiDenseTracker::IsKeypointValid(const cv::KeyPoint &kp,
                                       uint32_t image_width,
                                       uint32_t image_height)
{
  // Only for the car dataset.
//  if (kp.pt.y > 400) {
//    return false;
//  }

   if (kp.response < 200 || (kp.angle / kp.response) > 3.0
       /*tracker_options_.harris_score_threshold*/) {
      return false;
   }

  uint32_t margin =
      tracker_options_.patch_dim * tracker_options_.pyramid_levels;
  // Key keypoint also can't be closer than a certain amount to the edge.
  if (kp.pt.x < margin || kp.pt.y < margin ||
      kp.pt.x > image_width - margin || kp.pt.y > image_height - margin)
  {
    return false;
  }

  // Make sure this keypoint isn't already ina a patch somewhere.
  if (mask_.GetMask(0, kp.pt.x, kp.pt.y))
  {
    return false;
  }

  return true;
}

bool SemiDenseTracker::IsReprojectionValid(const Eigen::Vector2t &pix,
                                           const cv::Mat& image)
{
  if (pix[0] <= 2 || pix[0] > (image.cols - 2) ) {
    return false;
  }

  if (pix[1] <= 2 || pix[1] > (image.rows - 2) ) {
    return false;
  }

  return true;
}

void SemiDenseTracker::BackProjectTrack(std::shared_ptr<DenseTrack> track,
                                        bool initialize_pixel_vals,
                                        uint32_t cam_id)
{
  DenseKeypoint& kp = track->ref_keypoint;
  // Unproject the center pixel for this track.
  kp.ray =
      camera_rig_->cameras_[cam_id]->Unproject(kp.center_px).normalized() *
      tracker_options_.default_ray_depth;

  //std::cerr << "Initializing keypoint at " << kp.pt.x << ", " <<
  //               kp.pt.y << " with response: " << kp.response << std::endl;


  for (uint32_t ii = 0 ; ii < tracker_options_.pyramid_levels ; ++ii) {

    Patch& patch = kp.patch_pyramid[ii];
    // Transform the patch to lower levels
    patch.center[0] = kp.center_px[0] * pyramid_coord_ratio_[ii][0];
    patch.center[1] = kp.center_px[1] * pyramid_coord_ratio_[ii][1];

    uint32_t array_dim = 0;
    double mean_value = 0;
    const double extent = (patch.dim - 1) / 2.0;
    const double x_max = patch.center[0] + extent;
    const double y_max = patch.center[1] + extent;

    for (double yy = patch.center[1] - extent; yy <= y_max ; ++yy) {
      for (double xx = patch.center[0] - extent; xx <= x_max ; ++xx) {
        // Calculate this pixel in the 0th level image
        Eigen::Vector2t px_level0(xx / pyramid_coord_ratio_[ii][0],
            yy / pyramid_coord_ratio_[ii][1]);
        patch.rays[array_dim] =
            camera_rig_->cameras_[cam_id]->Unproject(px_level0).normalized() *
            tracker_options_.default_ray_depth;
        if (initialize_pixel_vals) {
          const double val = GetSubPix(image_pyramid_[cam_id][ii], xx, yy);
          patch.values[array_dim] = val;
          track->transfer[cam_id].projected_values[array_dim] =
              patch.values[array_dim];
          track->transfer[cam_id].projections[array_dim] = px_level0;
          mean_value += patch.values[array_dim];
        }
        array_dim++;
      }
    }

//    std::cerr << "\tCenter at level " << ii << " " << patch.center[0] <<
//                 ", " << patch.center[1] << " x: " <<
//                 patch.center[0] - extent << " to " << x_max << " y: " <<
//                 patch.center[1] - extent << " to " << y_max << std::endl;

    // Calculate the mean, and subtract from all values.
    if (initialize_pixel_vals) {
      mean_value /= patch.values.size();
      patch.mean = mean_value;
    }

    if (array_dim != patch.rays.size()) {
      std::cerr << "Not enough rays!" << std::endl;
      throw 0;
    }
  }
}

uint32_t SemiDenseTracker::StartNewTracks(
    std::vector<cv::Mat> &image_pyrmaid,
    std::vector<cv::KeyPoint> &cv_keypoints,
    uint32_t num_to_start,
    uint32_t cam_id)
{
  // Initialize the random inverse depth generator.
  const double range = tracker_options_.default_rho * 0.1;
  std::uniform_real_distribution<double> distribution(
        tracker_options_.default_rho - range,
        tracker_options_.default_rho + range);

  uint32_t num_started = 0;
  // const CameraInterface& cam = *camera_rig_->cameras[0];
  previous_keypoints_.clear();
  previous_keypoints_.reserve(cv_keypoints.size());

  std::sort(cv_keypoints.begin(), cv_keypoints.end(),
            [](const cv::KeyPoint& a, const cv::KeyPoint& b)
  { return a.response > b.response; } );


  // Clear the new tracks array, which will be populated below.
  new_tracks_.clear();

  for (cv::KeyPoint& kp : cv_keypoints)
  {
    if (num_to_start == 0) {
      break;
    }

    // Figure out which feature cell this particular reprojection falls into,
    // and increment that cell
    const uint32_t addressx =
        (kp.pt.x / image_pyrmaid[0].cols) * tracker_options_.feature_cells;
    const uint32_t addressy =
        (kp.pt.y / image_pyrmaid[0].rows) * tracker_options_.feature_cells;
    if (feature_cells_(addressy, addressx) >= lm_per_cell_) {
      continue;
    }

    if (!IsKeypointValid(kp, image_pyrmaid[0].cols,
                         image_pyrmaid[0].rows)) {
      continue;
    }

    mask_.SetMask(0, kp.pt.x, kp.pt.y);
    feature_cells_(addressy, addressx)++;


    // Otherwise extract pyramid for this keypoint, and also backproject all
    // pixels as the rays will not change.
    std::shared_ptr<DenseTrack> new_track(
          new DenseTrack(tracker_options_.pyramid_levels, pyramid_patch_dims_,
                         num_cameras_));
    new_track->id = next_track_id_++;
    current_tracks_.push_back(new_track);
    new_tracks_.push_back(new_track);
    DenseKeypoint& new_kp = new_track->ref_keypoint;

    new_kp.response = kp.response;
    new_kp.response2 = kp.angle;
    new_kp.center_px = Eigen::Vector2t(kp.pt.x, kp.pt.y);

    new_track->keypoints.emplace_back(num_cameras_);
    Keypoint& track_kp = new_track->keypoints.back()[0];
    track_kp.kp = new_kp.center_px;
    track_kp.tracked = true;

    for (unsigned int ii = 1; ii < num_cameras_; ++ii) {
      new_track->keypoints.back()[ii].kp.setZero();
      new_track->keypoints.back()[ii].tracked = 0;
    }

    new_track->num_good_tracked_frames++;

    // inherit the depth of this track from the closeset track.
    bool seeded_from_closest_track = false;
    if (tracker_options_.use_closest_track_to_seed_rho) {
      std::shared_ptr<DenseTrack> closest_track = nullptr;
      double min_distance = DBL_MAX;
      for (std::shared_ptr<DenseTrack> track : current_tracks_) {
        const double dist =
            (track->keypoints.back()[0].kp - new_kp.center_px).norm();
        if (dist < min_distance && track->keypoints.size() > 2) {
          min_distance = dist;
          closest_track = track;
        }
      }

      if (closest_track != nullptr && min_distance < 100) {
        new_kp.rho = closest_track->ref_keypoint.rho;
        seeded_from_closest_track = true;
      }
    }

    if (!seeded_from_closest_track) {
      new_kp.rho = distribution(generator_);
    }

    BackProjectTrack(new_track, true);

    num_to_start--;
    num_started++;
  }
  return num_started;
}

double SemiDenseTracker::EvaluateTrackResiduals(uint32_t level,
    const std::vector<std::vector<cv::Mat>>& image_pyrmaid,
    std::list<std::shared_ptr<DenseTrack>>& tracks,
    bool transfer_jacobians,
    bool optimized_tracks_only)
{
  prev_feature_cells_ = feature_cells_;
  feature_cells_.setZero();
  feature_cell_rho_.setZero();

  double residual = 0;
  uint32_t residual_count = 0;
  const Sophus::SE3d t_vc = camera_rig_->t_wc_[0];
  for (unsigned int cam_id = 0 ; cam_id < num_cameras_ ; ++cam_id) {
    const Sophus::SE3d t_cv = camera_rig_->t_wc_[cam_id].inverse();
    for (std::shared_ptr<DenseTrack>& track : tracks) {
      if (optimized_tracks_only && !track->residual_used) {
        continue;
      }

      PatchTransfer& transfer = track->transfer[cam_id];
      const Sophus::SE3d track_t_ba = t_cv * t_ba_ * track->t_ba * t_vc;
      DenseKeypoint& ref_kp = track->ref_keypoint;
      Patch& ref_patch = ref_kp.patch_pyramid[level];

      uint32_t num_inliers = 0;

      TransferPatch(
            track, level, cam_id, track_t_ba,  camera_rig_->cameras_[cam_id],
            transfer, transfer_jacobians);

      transfer.tracked_pixels = 0;
      transfer.rmse = 0;
      double ncc_num = 0, ncc_den_a = 0, ncc_den_b = 0;

      if (transfer.valid_projections.size() < ref_patch.rays.size() / 2) {
        continue;
      }


      for (size_t kk = 0; kk < transfer.valid_rays.size() ; ++kk)
      {
        const size_t ii = transfer.valid_rays[kk];
        // First transfer this pixel over to our current image.
        const double val_pix = transfer.projected_values[ii];

        const Scalar c_huber = 1.2107 * pyramid_error_thresholds_[level];
        const double mean_s_ref = ref_patch.values[ii] - ref_patch.mean;
        const double mean_s_proj = val_pix - transfer.mean_value;
        double res = mean_s_proj - mean_s_ref;
        bool inlier = true;
        if (tracker_options_.use_robust_norm_) {
          const double weight_sqrt = //sqrt(1.0 / ref_patch.statistics[ii][1]);
              sqrt(fabs(res) > c_huber ? c_huber / fabs(res) : 1.0);
           // std::cerr << "Weight for " << res << " at level " << level << " is " <<
           //              weight_sqrt * weight_sqrt << std::endl;
          res *= weight_sqrt;
          if (weight_sqrt != 1) {
            inlier = false;
          }
        }
        const double res_sqr = res * res;

        if (inlier) {
          transfer.rmse += res_sqr;
          ncc_num += mean_s_ref * mean_s_proj;
          ncc_den_a += mean_s_ref * mean_s_ref;
          ncc_den_b += mean_s_proj * mean_s_proj;
          num_inliers++;
        }

        transfer.residuals[ii] = res;
        residual_count++;
        residual += res_sqr;

        transfer.tracked_pixels++;
      }

      // Compute the track RMSE and NCC scores.
      transfer.rmse = transfer.tracked_pixels == 0 ?
            1e9 : sqrt(transfer.rmse / num_inliers);
      const double denom = sqrt(ncc_den_a * ncc_den_b);
      transfer.ncc = denom == 0 ? 0 : ncc_num / denom;
    }
  }

  return sqrt(residual);
}

void SemiDenseTracker::ReprojectTrackCenters()
{
  average_track_length_ = 0;
  const Sophus::SE3d t_vc = camera_rig_->t_wc_[0];
  for (unsigned int cam_id = 0; cam_id < num_cameras_ ; ++cam_id) {
    const calibu::CameraInterface<Scalar>& cam = *camera_rig_->cameras_[cam_id];
    const Sophus::SE3d t_cv = camera_rig_->t_wc_[cam_id].inverse();

    for (std::shared_ptr<DenseTrack>& track : current_tracks_)
    {
      const Sophus::SE3d track_t_ba = t_cv * t_ba_ * track->t_ba * t_vc;
      const DenseKeypoint& ref_kp = track->ref_keypoint;
      // Transfer the center ray. This is used for 2d tracking.
      const Eigen::Vector2t center_pix =
          cam.Transfer3d(track_t_ba, ref_kp.ray, ref_kp.rho);
      if (IsReprojectionValid(center_pix, image_pyramid_[cam_id][0])) {
        track->keypoints.back()[cam_id].kp = center_pix;
        mask_.SetMask(0, center_pix[0], center_pix[1]);

        // Figure out which feature cell this particular reprojection falls into,
        // and increment that cell.
        const uint32_t addressx =
            (center_pix[0] / image_pyramid_[cam_id][0].cols) *
            tracker_options_.feature_cells;
        const uint32_t addressy =
            (center_pix[1] / image_pyramid_[cam_id][0].rows) *
            tracker_options_.feature_cells;
        if (addressy > feature_cells_.rows() ||
            addressx > feature_cells_.cols()) {
          std::cerr << "Out of bounds feature cell access at : " << addressy <<
                       ", " << addressx << std::endl;
        }
        feature_cells_(addressy, addressx)++;

        if (track->keypoints.size() > 1) {
          average_track_length_ +=
              (track->keypoints.back()[cam_id].kp -
               track->keypoints.front()[cam_id].kp).norm();
        }
      } else {
        // invalidate this latest keypoint.
        track->keypoints.back()[cam_id].tracked = false;
        track->transfer[cam_id].tracked_pixels = 0;
      }
    }
  }

  if (current_tracks_.size() > 0) {
    average_track_length_ /= current_tracks_.size();
  }
}

void SemiDenseTracker::TransformTrackTabs(const Sophus::SE3d &t_cb)
{
  // Multiply the t_ba of all tracks by the current delta.
  for (std::shared_ptr<DenseTrack>& track : current_tracks_)
  {
    track->t_ba = t_cb * track->t_ba;
  }
}

void SemiDenseTracker::OptimizeTracks(int level, bool optimize_landmarks,
                                      bool optimize_pose, bool trust_guess)
{
  const double time = Tic();
  OptimizationStats stats;
  OptimizationOptions options;
  bool roll_back = false;
  Sophus::SE3d t_ba_old_;
  int last_level = level;
  if (level == -1) {
    bool optimized_pose = false;
    // Initialize the dense matrices that will be used to solve the problem.
    for (int ii = tracker_options_.pyramid_levels - 1 ; ii >= 0 ; ii--) {
      last_level = ii;

      if (trust_guess) {
        options.optimize_landmarks = true;
        options.optimize_pose = false;
      } else {
        if (average_track_length_ > 5) {
          if (optimized_pose == false) {
            options.optimize_landmarks = false;
            options.optimize_pose = true;
            if (last_level < 3) {
              ii = tracker_options_.pyramid_levels;
              optimized_pose = true;
            }
          } else {
            options.optimize_landmarks = true;
            options.optimize_pose = !options.optimize_landmarks;
          }
        } else {
          options.optimize_landmarks = true;
          options.optimize_pose = true;
        }
      }

      uint32_t iterations = 0;
      do {
        //double time = Tic();
        t_ba_old_ = t_ba_;
        OptimizePyramidLevel(last_level, image_pyramid_, current_tracks_,
                             options, stats);

        double post_error = EvaluateTrackResiduals(
              last_level, image_pyramid_, current_tracks_, false, true);

        /*
        std::cerr << "Pyramid level " << last_level << " with p: " <<
                     options.optimize_pose << " l:" <<
                     options.optimize_landmarks << " took " << Toc(time) <<
                     " seconds. t:" << stats.transfer_time << " j:" <<
                     stats.jacobian_time << " s:" << stats.schur_time <<
                     " so:" << stats.solve_time << " l: " << stats.lm_time <<
                     " prev: " << stats.pre_solve_error << " post: " <<
                     post_error << std::endl;
                     */

        // std::cerr << "Level : " << last_level << " it: " << iterations <<
        //              " p: " << options.optimize_pose << " l: " <<
        //              options.optimize_landmarks << std::endl;

        // std::cerr << "deltap: " << stats.delta_pose_norm << " deltal: " <<
        //              stats.delta_lm_norm << " prev rmse: " <<
        //              stats.pre_solve_error << " post rmse: " <<
        //              post_error << std::endl;

        if (post_error > stats.pre_solve_error) {
//            std::cerr << "Exiting due to " << post_error << " > " <<
//                         stats.pre_solve_error << " rolling back. " <<
//                         std::endl;

            roll_back = true;
            break;
        }
        const double change = post_error == 0 ? 0 :
          fabs(stats.pre_solve_error - post_error) / post_error;
        if (change < 0.01) {
//            std::cerr << "Exiting due to change% = " << change <<
//                         " prev error " << stats.pre_solve_error <<
//                         " post : " << post_error << std::endl;
          break;
        }

        post_error = stats.pre_solve_error;
        iterations++;

      } while (stats.delta_pose_norm > 1e-4 || stats.delta_lm_norm >
               1e-4 * current_tracks_.size());
    }

//    std::list<std::shared_ptr<DenseTrack>> new_tracks;
//    for (std::shared_ptr<DenseTrack>& track : current_tracks_) {
//      if (track-) {
//        track->offset_2d.setZero();
//        new_tracks.push_back(track);
//      }
//    }
//    // Now that we've done 3d to 2d tracking, do 2d to 2d tracking for new
//    // tracks.
//    for (int ii = tracker_options_.pyramid_levels - 1 ; ii >= 0 ; ii--) {
//      Do2dAlignment(image_pyrmaid_, new_tracks, ii);
//    }


  } else {
    options.optimize_landmarks = optimize_landmarks;
    options.optimize_pose = optimize_pose;
    t_ba_old_ = t_ba_;
    OptimizePyramidLevel(level, image_pyramid_, current_tracks_,
                         options, stats);
    ///zzzzzz evaluate residuals at all levels so we can see
    for (uint32_t ii = 0 ; ii < tracker_options_.pyramid_levels ; ++ii)  {
      if (ii != (uint32_t)level) {
        std::cerr << "post rmse: " << ii << " " << EvaluateTrackResiduals(
                       ii, image_pyramid_, current_tracks_, false, true) <<
                     std::endl;
      }
    }
    double post_error = EvaluateTrackResiduals(
          level, image_pyramid_, current_tracks_, false, true);
    std::cerr << "post rmse: " << post_error << " " << "pre : " <<
                 stats.pre_solve_error << std::endl;
    if (post_error > stats.pre_solve_error) {
        std::cerr << "Exiting due to " << post_error << " > " <<
                     stats.pre_solve_error << " rolling back. " <<
                     std::endl;

        roll_back = true;
    }
  }

  if (roll_back) {
    // Roll back the changes.
    t_ba_ = t_ba_old_;
    for (std::shared_ptr<DenseTrack> track : current_tracks_) {
     // Only roll back tracks that were in the optimization.
     if (track->opt_id == UINT_MAX) {
       continue;
     }  
     track->ref_keypoint.rho = track->ref_keypoint.old_rho;

    }

    double post_error = EvaluateTrackResiduals(
          last_level, image_pyramid_, current_tracks_, false, true);
  }

  // Reproject patch centers. This will add to the keypoints vector in each
  // patch which is used to pass on center values to an outside 2d BA.
  ReprojectTrackCenters();


  // Print pre-post errors
  std::cerr << "Level " << level << " solve took " << Toc(time) << "s" <<
               " with delta_p_norm: " << stats.delta_pose_norm << " and delta "
               "lm norm: " << stats.delta_lm_norm << std::endl;
}

void SemiDenseTracker::PruneTracks()
{
  if (image_pyramid_.size() == 0) {
    return;
  }

  num_successful_tracks_ = 0;
  // OptimizeTracks();

  std::list<std::shared_ptr<DenseTrack>>::iterator iter =
      current_tracks_.begin();
  while (iter != current_tracks_.end()) {
    std::shared_ptr<DenseTrack> track = *iter;

    uint32_t num_successful_cams = 0;
    for (unsigned int cam_id = 0; cam_id < num_cameras_ ; ++cam_id) {
      PatchTransfer& transfer = track->transfer[cam_id];
      const double dim_ratio = transfer.dimension / tracker_options_.patch_dim;
      const double percent_tracked =
          ((double)transfer.tracked_pixels /
           (double)transfer.pixels_attempted);

      if (transfer.ncc > tracker_options_.dense_ncc_threshold &&
          percent_tracked == 1.0 && !(track->transfer[cam_id].level == 0 &&
                                      (dim_ratio > 2.0 || dim_ratio < 0.5))) {
        track->keypoints.back()[cam_id].tracked = true;
        num_successful_cams++;
      }
    }

    if (num_successful_cams == 0) {
      track->tracked = false;
      iter = current_tracks_.erase(iter);
    } else {
      track->num_good_tracked_frames++;
      track->tracked = true;
      num_successful_tracks_++;
      ++iter;
    }
  }
}

void SemiDenseTracker::PruneOutliers()
{
  num_successful_tracks_ = 0;
  std::list<std::shared_ptr<DenseTrack>>::iterator iter =
      current_tracks_.begin();
  while (iter != current_tracks_.end())
  {
    std::shared_ptr<DenseTrack> track = *iter;
    if (track->is_outlier) {
      iter = current_tracks_.erase(iter);
    } else {
      num_successful_tracks_ ++;
      ++iter;
    }
  }
}


void SemiDenseTracker::TransferPatch(std::shared_ptr<DenseTrack> track,
                                     uint32_t level,
                                     uint32_t cam_id,
                                     const Sophus::SE3d &t_ba,
                                     calibu::CameraInterface<Scalar> *cam,
                                     PatchTransfer& result,
                                     bool transfer_jacobians,
                                     bool use_approximation)
{
  result.level = level;
  Eigen::Vector4t ray;
  DenseKeypoint& ref_kp = track->ref_keypoint;
  Patch& ref_patch = ref_kp.patch_pyramid[level];
  result.valid_projections.clear();
  result.valid_rays.clear();
  result.dprojections.clear();
  result.valid_projections.reserve(ref_patch.rays.size());
  result.valid_rays.reserve(ref_patch.rays.size());
  result.dprojections.reserve(ref_patch.rays.size());

  result.projected_values.resize(ref_patch.rays.size());
  result.residuals.resize(ref_patch.rays.size());
  result.pixels_attempted = 0;

  result.mean_value = 0;

  // If we are doing simplified (4 corner) patch transfer, transfer the four
  // corners.
  bool corners_project = true;
  Eigen::Vector2t corner_projections[4];
  Eigen::Matrix<double, 2, 4> corner_dprojections[4];
  for (int ii = 0 ; ii < 4 ; ++ii) {
    const Eigen::Vector3t& corner_ray =
        ref_patch.rays[pyramid_patch_corner_dims_[level][ii]];
    corner_projections[ii] = cam->Transfer3d(t_ba, corner_ray, ref_kp.rho) +
        track->offset_2d;

    if (transfer_jacobians) {
      ray.head<3>() = corner_ray;
      ray[3] = ref_kp.rho;
      const Eigen::Vector4t ray_b = MultHomogeneous(t_ba, ray);
      corner_dprojections[ii] =
          cam->dTransfer3d_dray(Sophus::SE3d(), ray_b.head<3>(), ray_b[3]);
    }
  }

  if (!corners_project) {
    return;
  }

  // Get the new dimension in pixels.
  result.dimension = (corner_projections[1] - corner_projections[0]).norm();


  // Transfer the center ray.
  const Eigen::Vector3t center_ray = ref_kp.ray;
  // Reproject the center and form a residual.
  result.center_projection =
      cam->Transfer3d(Sophus::SE3d(), center_ray, ref_kp.rho);

  // If the user requested jacobians, get the center ray jacobian in the ref
  // frame.
  if (transfer_jacobians) {
    result.center_dprojection =
        cam->dTransfer3d_dray(
          Sophus::SE3d(), center_ray, ref_kp.rho).topLeftCorner<2, 3>();
  }

  // First project the entire patch and see if it falls within the bounds of
  // the image.
  for (size_t ii = 0; ii < ref_patch.rays.size() ; ++ii) {
    const double tl_factor = pyramid_patch_interp_factors_[level][ii][0];
    const double tr_factor = pyramid_patch_interp_factors_[level][ii][1];
    const double bl_factor = pyramid_patch_interp_factors_[level][ii][2];
    const double br_factor = pyramid_patch_interp_factors_[level][ii][3];

    // First transfer this pixel over to our current image.
    Eigen::Vector2t pix;
    if (corners_project) {
      if (!use_approximation) {
        pix = cam->Transfer3d(
              t_ba, ref_patch.rays[ii], ref_kp.rho) + track->offset_2d;
      } else {
        // std::cerr << "prev pix: " << pix.transpose() << std::endl;
        // linearly interpolate this
        pix =
            tl_factor * corner_projections[0] +
            tr_factor * corner_projections[1] +
            bl_factor * corner_projections[2] +
            br_factor * corner_projections[3];
        // std::cerr << "post pix: " << pix.transpose() << "\n" << std::endl;
      }
    }

    result.pixels_attempted++;
    // Check bounds
    result.projections[ii] = pix;
    pix[0] *= pyramid_coord_ratio_[level][0];
    pix[1] *= pyramid_coord_ratio_[level][1];

    if (!IsReprojectionValid(pix, image_pyramid_[cam_id][level])) {
      // std::cerr << "Reprojection at " << pix.transpose() <<
      //              " not valid. " << std::endl;
      result.projected_values[ii] = 0;
      result.residuals[ii] = 0;
      continue;
    } else {
      // ray.head<3>() = ref_patch.rays[ii];
      // ray[3] = ref_kp.rho;
      // const Eigen::Vector4t ray_b = Sophus::MultHomogeneous(t_ba, ray);
      if (corners_project && transfer_jacobians) {
        ray.head<3>() = ref_patch.rays[ii];
        ray[3] = ref_kp.rho;
        const Eigen::Vector4t ray_b = MultHomogeneous(t_ba, ray);
        if (!use_approximation) {
          result.dprojections.push_back(
                cam->dTransfer3d_dray(
                  Sophus::SE3d(), ray_b.head<3>(), ray_b[3]));
        } else {
//         std::cerr << "Prev dprojection: " << std::endl << dprojection_dray <<
//                      std::endl;
          result.dprojections.push_back(
                tl_factor * corner_dprojections[0] +
              tr_factor * corner_dprojections[1] +
              bl_factor * corner_dprojections[2] +
              br_factor * corner_dprojections[3]
              );
        }
        //  std::cerr << "new dprojection: " << std::endl <<
        //               result.dprojections.back() << "\n" <<  std::endl;
      }

      result.valid_projections.push_back(pix);
      result.valid_rays.push_back(ii);
      if (std::isnan(pix[0]) || std::isnan(pix[1])) {
        std::cerr << "Pixel at: " << pix.transpose() <<
                     " center ray: " << ref_kp.ray << " rho: " <<
                     ref_kp.rho << std::endl;
      }
      const double val = GetSubPix(image_pyramid_[cam_id][level],
                                   pix[0], pix[1]);
      result.projected_values[ii] = val;
      result.mean_value += val;
    }
  }

  // Calculate the mean value
  if (result.valid_rays.size()) {
    result.mean_value /= result.valid_rays.size();
  } else {
    result.mean_value = 0;
  }
  ref_patch.projected_mean = result.mean_value;
}

void SemiDenseTracker::StartNewLandmarks()
{
  // Figure out the number of landmarks per cell that's required. For this we
  // will look at the previous cells to see how many active cells there were.
  const uint32_t num_cells = powi(tracker_options_.feature_cells, 2);
//  const double default_lm_per_cell =
//      (double) tracker_options_.num_active_tracks / num_cells;
//  uint32_t num_active_cells = 0;
//  for (int ii = 0; ii < feature_cells_.rows() ; ++ii) {
//    for (int jj = 0; jj < feature_cells_.cols() ; ++jj) {
//      if (feature_cells_(ii, jj) > default_lm_per_cell / 2.0) {
//        num_active_cells++;
//      }
//    }
//  }
//  if (num_active_cells == 0) {
//    num_active_cells = powi(tracker_options_.feature_cells, 2);
//  }
  lm_per_cell_ = tracker_options_.num_active_tracks / num_cells;

  //std::cerr << "Num active cells " << num_active_cells << " lm per cell " <<
  //             lm_per_cell_ << std::endl;

  std::vector<cv::KeyPoint> cv_keypoints;
  // Extract features and descriptors from this image
  ExtractKeypoints(image_pyramid_[0][0], cv_keypoints);


  // Afterwards, we must spawn a number of new tracks
  const int num_new_tracks =
      std::max(0, (int)tracker_options_.num_active_tracks -
               (int)num_successful_tracks_);

  const uint32_t started =
      StartNewTracks(image_pyramid_[0], cv_keypoints, num_new_tracks, 0);

  std::cerr << "Tracked: " << num_successful_tracks_ << " started " <<
               started << " out of " << num_new_tracks << " new tracks with " <<
               cv_keypoints.size() << " keypoints " << std::endl;
}

void SemiDenseTracker::GetImageDerivative(
    const cv::Mat& image, const Eigen::Vector2d& pix,
    Eigen::Matrix<double, 1, 2>& di_dppix, double val_pix)
{
  double eps = 1e-9;
  const double valx_pix = GetSubPix(image, pix[0] + eps, pix[1]);
  const double valy_pix = GetSubPix(image, pix[0], pix[1] + eps);
  di_dppix[0] = (valx_pix - val_pix)/(eps);
  di_dppix[1] = (valy_pix - val_pix)/(eps);
}

void SemiDenseTracker::Do2dAlignment(
    const std::vector<std::vector<cv::Mat>>& image_pyrmaid,
    std::list<std::shared_ptr<DenseTrack>> &tracks,
    uint32_t level)
{

  Eigen::LDLT<Eigen::Matrix2d> solver;
  Eigen::Matrix2d jtj;
  Eigen::Vector2d jtr, delta_pix, delta_pix_0th_level;
  double res_total;
  Eigen::Matrix<double, 1, 2> di_dp;
  double ncc_num = 0, ncc_den_a = 0, ncc_den_b = 0;
  const Sophus::SE3d t_vc = camera_rig_->t_wc_[0];
  for (int cam_id = 0; cam_id < num_cameras_; ++cam_id) {
    const Sophus::SE3d t_cv = camera_rig_->t_wc_[cam_id].inverse();
    for (std::shared_ptr<DenseTrack>& track : tracks) {
      if (track->keypoints.size() < 2) {
        continue;
      }

      while (true) {
        jtj.setZero();
        jtr.setZero();
        res_total = 0;
        ncc_num = 0;
        ncc_den_a = 0;
        ncc_den_b = 0;

        // Set up the 2d alignment problem.
        PatchTransfer& transfer = track->transfer[cam_id];

        if (transfer.level != level) {
          // We have to re-transfer this track.
          const Sophus::SE3d track_t_ba = t_cv * t_ba_ * track->t_ba * t_vc;
          TransferPatch(track, level, cam_id,  track_t_ba,
                        camera_rig_->cameras_[cam_id], transfer, false);
        }
        // uint32_t level = transfer.level;
        DenseKeypoint& ref_kp = track->ref_keypoint;
        Patch& ref_patch = ref_kp.patch_pyramid[level];
        bool out_of_bounds = false;
        for (size_t kk = 0; kk < transfer.valid_rays.size() ; ++kk) {
          const size_t ii = transfer.valid_rays[kk];
          const Eigen::Vector2t& pix = transfer.valid_projections[kk];
          if (!IsReprojectionValid(pix, image_pyramid_[cam_id][level])) {
            out_of_bounds = true;
            break;
          }
          // Get the residual at this point.
          const double val_pix = transfer.projected_values[ii];
              //GetSubPix(image_pyrmaid[level], pix[0], pix[1]);
          const double mean_s_ref = ref_patch.values[ii] - ref_patch.mean;
          const double mean_s_proj = val_pix - transfer.mean_value;
          double res = mean_s_proj - mean_s_ref;

          // Also tet the jacobian.
          GetImageDerivative(image_pyrmaid[cam_id][level], pix, di_dp, val_pix);

          jtj += di_dp.transpose() * di_dp;
          jtr += di_dp.transpose() * res;
          res_total += fabs(res);
        }

        if (out_of_bounds) {
          break;
        }

        // Calculate the update to this patch.
        solver.compute(jtj);
        delta_pix = solver.solve(jtr);
        delta_pix_0th_level[0] = delta_pix[0] / pyramid_coord_ratio_[level][0];
        delta_pix_0th_level[1] = delta_pix[1] / pyramid_coord_ratio_[level][1];
        track->offset_2d += delta_pix;

        out_of_bounds = false;
        transfer.mean_value = 0;
        for (size_t kk = 0; kk < transfer.valid_rays.size() ; ++kk) {
          const size_t ii = transfer.valid_rays[kk];
          transfer.valid_projections[kk] -= delta_pix;
          transfer.projections[ii] -= delta_pix_0th_level;
          const Eigen::Vector2t& pix = transfer.valid_projections[kk];
          if (!IsReprojectionValid(pix, image_pyramid_[cam_id][level])) {
            out_of_bounds = true;
            break;
          }
          transfer.projected_values[ii] = GetSubPix(image_pyrmaid[cam_id][level],
                                                     pix[0], pix[1]);
          transfer.mean_value += transfer.projected_values[ii];
        }

        if (out_of_bounds) {
          break;
        }
        if (transfer.valid_rays.size() != 0) {
          transfer.mean_value /= transfer.valid_rays.size();
        }

        for (size_t kk = 0; kk < transfer.valid_rays.size() ; ++kk) {
          const size_t ii = transfer.valid_rays[kk];
          const double mean_s_ref = ref_patch.values[ii] - ref_patch.mean;
          const double mean_s_proj = transfer.projected_values[ii] -
              transfer.mean_value;
          ncc_num += mean_s_ref * mean_s_proj;
          ncc_den_a += mean_s_ref * mean_s_ref;
          ncc_den_b += mean_s_proj * mean_s_proj;
          transfer.residuals[ii] = mean_s_proj - mean_s_ref;
        }

        const double denom = sqrt(ncc_den_a * ncc_den_b);
        const double prev_ncc = transfer.ncc;
        transfer.ncc = denom == 0 ? 0 : ncc_num / denom;

        // std::cerr << "Updated track " << track->id << " using 2d alignment of " <<
        //              delta_pix.transpose() << " pixels. r: " << res_total <<
        //              res_total << std::endl;

        if (transfer.ncc <= prev_ncc) {
          // roll back the changes
          transfer.ncc = prev_ncc;
          break;
        } else {
          track->keypoints.back()[cam_id].kp -= delta_pix;
        }

  //      if ((track->ncc - prev_ncc) / prev_ncc < 0.01) {
  //        break;
  //      }
        if (delta_pix.norm() < 0.01) {
          break;
        }

      }
    }
  }
}

void SemiDenseTracker::OptimizePyramidLevel(uint32_t level,
    const std::vector<std::vector<cv::Mat>>& image_pyrmaid,
    std::list<std::shared_ptr<DenseTrack>> &tracks,
    const OptimizationOptions& options,
    OptimizationStats& stats)
{
  static Eigen::Matrix<double, 6, 6> u;
  static Eigen::Matrix<double, 6, 1> r_p;
  u.setZero();
  r_p.setZero();
  Eigen::Matrix<double, 6, 1> w;
  static std::vector<Eigen::Matrix<double, 6, 1>> w_vec;
  double v;
  static std::vector<double> v_inv_vec;
  double r_l;
  Eigen::VectorXd r_l_vec(tracks.size());

  v_inv_vec.resize(tracks.size());
  w_vec.resize(v_inv_vec.size());

  Eigen::Matrix<double, 2, 6> dp_dx;
  Eigen::Matrix<double, 2, 4> dp_dray;
  Eigen::Vector4t ray;
  Eigen::Matrix<double, 2, 4> dprojection_dray;
  static std::vector<Eigen::Matrix<double, 1, 6>> di_dx;
  static std::vector<double> di_dray;
  static std::vector<double> res;
  di_dx.clear();
  di_dray.clear();
  res.clear();
  Eigen::Matrix<double, 1, 6> mean_di_dx;
  double mean_di_dray;
  Eigen::Matrix<double, 1, 6> final_di_dx;
  double final_di_dray;
  // std::vector<Eigen::Vector2t> valid_projections;
  // std::vector<unsigned int> valid_rays;

  uint32_t track_id = 0;
  uint32_t residual_id = 0;
  uint32_t num_inliers = 0;


  double track_residual;
  double residual = 0;
  uint32_t residual_count = 0;
  uint32_t residual_offset = 0;

  // First project all tracks into this frame and form
  // the localization step.
  stats.jacobian_time = 0;
  stats.transfer_time = 0;
  stats.schur_time = 0;
  stats.solve_time = 0;
  stats.lm_time = 0;
  double schur_time;

  if (tracks.size() == 0) {
    stats.pre_solve_error = 0;
    stats.delta_pose_norm = 0;
    stats.delta_lm_norm = 0;
    return;
  }

  const Sophus::SE3d t_vc = camera_rig_->t_wc_[0];
  for (std::shared_ptr<DenseTrack>& track : tracks) {
    track->opt_id = UINT_MAX;
    track->residual_used = false;
    // If we are not solving for landmarks, there is no point including
    // uninitialized landmarks in the camera pose estimation
    if (options.optimize_landmarks == 0 &&
        track->keypoints.size() < MIN_OBS_FOR_CAM_LOCALIZATION) {
      continue;
    }

    track_residual = 0;

    // Project into the image and form the problem.
    DenseKeypoint& ref_kp = track->ref_keypoint;
    Patch& ref_patch = ref_kp.patch_pyramid[level];

    // Prepare the w matrix. We will add to it as we go through the rays.
    w.setZero();
    // Same for the v matrix
    v = 0;
    // Same for the RHS subtraction term
    r_l = 0;

    track->residual_offset = residual_offset;
    residual_offset++;

    for (int cam_id = 0 ; cam_id < num_cameras_ ; ++cam_id) {
      const Sophus::SE3d t_cv = camera_rig_->t_wc_[cam_id].inverse();
      const Eigen::Matrix4d t_cv_mat = t_cv.matrix();

      const Sophus::SE3d track_t_va = t_ba_ * track->t_ba * t_vc;
      const Sophus::SE3d track_t_ba = t_cv * track_t_va;
      const Eigen::Matrix4d track_t_ba_matrix = track_t_ba.matrix();

      PatchTransfer& transfer = track->transfer[cam_id];
      transfer.tracked_pixels = 0;
      transfer.rmse = 0;

      const double transfer_time = Tic();
      if (options.transfer_patches) {
        TransferPatch(track, level, cam_id, track_t_ba,
                      camera_rig_->cameras_[cam_id], transfer, true);
      }
      stats.transfer_time += Toc(transfer_time);

      // Do not use this patch if less than half of its pixels reprojcet.
      if (transfer.valid_projections.size() < ref_patch.rays.size() / 2) {
        // std::cerr << "Rejecting track due to lack of projections." << std::endl;
        continue;
      }

      const double jacobian_time = Tic();
      di_dx.resize(transfer.valid_rays.size());
      di_dray.resize(transfer.valid_rays.size());
      res.resize(transfer.valid_rays.size());
      mean_di_dray = 0;
      mean_di_dx.setZero();
      double ncc_num = 0, ncc_den_a = 0, ncc_den_b = 0;
      for (size_t kk = 0; kk < transfer.valid_rays.size() ; ++kk) {
        const size_t ii = transfer.valid_rays[kk];
        // First transfer this pixel over to our current image.
        const Eigen::Vector2t& pix = transfer.valid_projections[kk];

        // need 2x6 transfer residual
        ray.head<3>() = ref_patch.rays[ii];
        ray[3] = ref_kp.rho;
        const Eigen::Vector4t ray_v = MultHomogeneous(track_t_va, ray);

        dprojection_dray = transfer.dprojections[kk];
        dprojection_dray *= pyramid_coord_ratio_[level][0];

        Eigen::Matrix<double, 1, 2> di_dp;
        const double val_pix = transfer.projected_values[ii];
        GetImageDerivative(image_pyrmaid[cam_id][level], pix, di_dp, val_pix);

        // need 2x4 transfer w.r.t. reference ray
        di_dray[kk] = di_dp * dp_dray.col(3);
        dp_dray = dprojection_dray * track_t_ba_matrix;

  //      for (unsigned int jj = 0; jj < 6; ++jj) {
  //        dp_dx.block<2,1>(0,jj) =
  //            //dprojection_dray * Sophus::SE3d::generator(jj) * ray;
  //            dprojection_dray * generators_[jj] * ray_v;
  //      }

        if (options.optimize_pose) {
          dprojection_dray *= t_cv_mat;
          dp_dx.col(0) = dprojection_dray.col(0) * ray_v[3];
          dp_dx.col(1) = dprojection_dray.col(1) * ray_v[3];
          dp_dx.col(2) = dprojection_dray.col(2) * ray_v[3];

          dp_dx.col(3) = dprojection_dray.col(2) * ray_v[1] -
                         dprojection_dray.col(1) * ray_v[2];

          dp_dx.col(4) = dprojection_dray.col(0) * ray_v[2] -
                         dprojection_dray.col(2) * ray_v[0];

          dp_dx.col(5) = dprojection_dray.col(1) * ray_v[0] -
                         dprojection_dray.col(0) * ray_v[1];

          di_dx[kk] = di_dp * dp_dx;
        }

        // Insert the residual.
        const Scalar c_huber = 1.2107 * pyramid_error_thresholds_[level];
        const double mean_s_ref = ref_patch.values[ii] - ref_patch.mean;
        const double mean_s_proj = val_pix - transfer.mean_value;
        res[kk] = mean_s_proj - mean_s_ref;
        bool inlier = true;
        if (tracker_options_.use_robust_norm_) {
          const double weight_sqrt = //sqrt(1.0 / ref_patch.statistics[ii][1]);
            sqrt(fabs(res[kk]) > c_huber ? c_huber / fabs(res[kk]) : 1.0);
           // std::cerr << "Weight for " << res[kk] << " at level " << level <<
           //              " is " << weight_sqrt * weight_sqrt << std::endl;
          res[kk] *= weight_sqrt;
          di_dx[kk] *= weight_sqrt;
          di_dray[kk] *= weight_sqrt;
          if (weight_sqrt != 1) {
            inlier = false;
          }
        }
        const double res_sqr = res[kk] * res[kk];

        if (inlier) {
          transfer.rmse += res_sqr;
          ncc_num += mean_s_ref * mean_s_proj;
          ncc_den_a += mean_s_ref * mean_s_ref;
          ncc_den_b += mean_s_proj * mean_s_proj;
          num_inliers++;
        }

        mean_di_dray += di_dray[kk];
        mean_di_dx += di_dx[kk];

        transfer.residuals[ii] = res[kk];
        residual_count++;
        track_residual += res_sqr;

        transfer.tracked_pixels++;
      }

      mean_di_dray /= transfer.valid_rays.size();
      mean_di_dx /= transfer.valid_rays.size();
      stats.jacobian_time += Toc(jacobian_time);

      // Test the jacobians calculated for this patch.
  //    std::vector<double> di_dray_fd(ref_patch.projected_values);
  //    if (options.optimize_landmarks) {
  //      double eps = 1e-9;
  //      double rho_backup = track->ref_keypoint.rho;

  //      track->ref_keypoint.rho = rho_backup + eps;
  //      PatchTransfer plus_transfer;
  //      TransferPatch(track, level, track_t_ba, camera_rig_->cameras_[0],
  //          plus_transfer, false, false);
  //      di_dray_fd = ref_patch.projected_values;

  //      track->ref_keypoint.rho = rho_backup - eps;
  //      PatchTransfer minus_transfer;
  //      TransferPatch(track, level, track_t_ba, camera_rig_->cameras_[0],
  //          minus_transfer, false, false);
  //      // Now calculate the jacobian
  //      for (size_t kk = 0; kk < di_dray_fd.size() ; ++kk) {
  //        di_dray_fd[kk] = ((di_dray_fd[kk] - plus_transfer.mean_value) -
  //            (ref_patch.projected_values[kk] - minus_transfer.mean_value)) /
  //            (2 * eps);
  //      }
  //    }


      schur_time = Tic();
      for (size_t kk = 0; kk < transfer.valid_rays.size() ; ++kk)
      {
        if (options.optimize_landmarks) {
          final_di_dray = di_dray[kk] - mean_di_dray;
  //        std::cerr << "level " << level << " di_dray " << final_di_dray <<
  //                     " di_dray_fd " << di_dray_fd[transfer.valid_rays[kk]] <<
  //                     std::endl;
          const double di_dray_id = final_di_dray;
          // Add the contribution of this ray to the w and v matrices.
          if (options.optimize_pose) {
            w += final_di_dx.transpose() * di_dray_id;
          }

          v += di_dray_id * di_dray_id;
          // Add contribution for the subraction term on the rhs.
          r_l += di_dray_id * res[kk];
        }

        if (options.optimize_pose) {
          final_di_dx = di_dx[kk] - mean_di_dx;
          // Update u by adding j_p' * j_p
          u += final_di_dx.transpose() * final_di_dx;
          // Update rp by adding j_p' * r
          r_p += final_di_dx.transpose() * res[kk];
        }

        residual_id++;
      }

      // Compute the track RMSE and NCC scores.
      transfer.rmse = transfer.tracked_pixels == 0 ?
            1e9 : sqrt(transfer.rmse / num_inliers);
      const double denom = sqrt(ncc_den_a * ncc_den_b);
      transfer.ncc = denom == 0 ? 0 : ncc_num / denom;
    }

    bool omit_track = false;
    if (track->id == longest_track_id_ && track->keypoints.size() <= 2 &&
        options.optimize_landmarks && options.optimize_pose &&
        num_cameras_ == 1) {
      std::cerr << "omitting longest track id " << longest_track_id_ <<
                   std::endl;
      omit_track = true;
    }

    // If this landmark is the longest track, we omit it to fix scale.
    if (options.optimize_landmarks && !omit_track) {
      track->opt_id = track_id;
      double regularizer = options.optimize_landmarks && options.optimize_pose ?
            1e3 : 0;//level >= 2 ? 1e3 : level == 1 ? 1e2 : 1e1;

      v += regularizer;
      if (v < 1e-6) {
        v = 1e-6;
      }

      if (std::isnan(v) || std::isinf(v)) {
        std::cerr << "v is bad: " << v << std::endl;
      }

      // std::cerr << "v: " << v << std::endl;
      const double v_inv = 1.0 / v;
      v_inv_vec[track->opt_id] = v_inv;
      r_l_vec(track->residual_offset) = r_l;

      if (options.optimize_pose) {
        w_vec[track->opt_id] = w;
        // Subtract the contribution of these residuals from u and r_p
        u -= w * v_inv * w.transpose();
        r_p -= w * v_inv * r_l;
      }
      track_id++;
    } else {
      track->opt_id = UINT_MAX;
    }

    // Add to the overal residual here, as we're sure the track will be
    // included in the optimization.
    residual += track_residual;
    track->residual_used = true;

    stats.schur_time += Toc(schur_time);
    // std::cerr << "track rmse for level " << level << " : " << track->rmse <<
    //              std::endl;
  }


  // Solve for the pose update
  const double solve_time = Tic();
  Eigen::Matrix<double, 6, 1> delta_p;
  if (options.optimize_pose) {
    Eigen::LDLT<Eigen::Matrix<Scalar, 6, 6>> solver;
    solver.compute(u);
    delta_p = solver.solve(r_p);
    t_ba_ = Sophus::SE3d::exp(-delta_p * tracker_options_.gn_scaling) * t_ba_;
  }
  stats.solve_time += Toc(solve_time);

  const double lm_time = Tic();
  stats.delta_lm_norm = 0;
  uint32_t delta_lm_count = 0;
  // Now back-substitute all the keypoints
  if (options.optimize_landmarks) {
    for (std::shared_ptr<DenseTrack>& track : tracks)
    {
      if (track->opt_id != UINT_MAX)
      {
        delta_lm_count++;
        double delta_ray;
        if (options.optimize_pose) {
          delta_ray = v_inv_vec[track->opt_id] *
            (r_l_vec(track->residual_offset) -
             w_vec[track->opt_id].transpose() * delta_p);
        } else {
          delta_ray = v_inv_vec[track->opt_id] *
              (r_l_vec(track->residual_offset));
        }

        delta_ray *= tracker_options_.gn_scaling;
        track->ref_keypoint.old_rho = track->ref_keypoint.rho;
        track->ref_keypoint.rho -= delta_ray;
        stats.delta_lm_norm += fabs(delta_ray);
  //          if (track->keypoints.size() == 2) {
  //            std::cerr << "Update to new track " << track->id << " : " <<
  //                         delta_ray << " vinv " << v_inv_vec[track->opt_id] <<
  //                         std::endl;
  //          }
        if (std::isnan(delta_ray) || std::isinf(delta_ray)) {
          std::cerr << "delta_ray " << track->id << ": " << delta_ray <<
                       "vinv:" << v_inv_vec[track->opt_id] << " r_l " <<
                       r_l_vec(track->residual_offset) << " w: " <<
                       w_vec[track->opt_id].transpose() << "dp : " <<
                       delta_p.transpose() << std::endl;
        }

        if (track->ref_keypoint.rho < 0) {
  //            std::cerr << "rho negative: " << track->ref_keypoint.rho <<
  //                         " from : " << track->ref_keypoint.old_rho <<
  //                         " id " << track->id << " kps " <<
  //                         track->keypoints.size() << " v_inv " <<
  //                         v_inv_vec[track->opt_id] << std::endl;
          track->ref_keypoint.rho = track->ref_keypoint.old_rho / 2;
        }
      }
    }
  }
  stats.lm_time += Toc(lm_time);

  // set the optimization stats.
  stats.pre_solve_error = sqrt(residual);
  stats.delta_pose_norm = delta_p.norm();
  stats.delta_lm_norm =
      delta_lm_count == 0 ? 0 : stats.delta_lm_norm / delta_lm_count;
}


double SemiDenseTracker::GetSubPix(const cv::Mat &image, double x, double y)
{
  return Interpolate(x, y, image.data, image.cols, image.rows);
}

void SemiDenseTracker::AddImage(const std::vector<cv::Mat> &images,
                                const Sophus::SE3d &t_ba_guess)
{
  std::cout << "SDTrack: Image size: " << images[0].rows << "x" << images[0].cols << "step: " << images[0].step << "\n";
  // If there were any outliers (externally marked), now is the time to prune
  // them.
  PruneOutliers();

  // Clear out all 2d offsets for new tracks
  for (std::shared_ptr<DenseTrack>& track : current_tracks_) {
    track->offset_2d.setZero();
    for (int cam_id = 0; cam_id < num_cameras_ ; ++cam_id) {
      track->transfer[cam_id].level = UNINITIALIZED_TRANSFER;
    }
  }

  mask_.Clear();
  t_ba_ = t_ba_guess;
  // Create the image pyramid for the incoming image
  for (int cam_id = 0 ; cam_id < num_cameras_ ; ++cam_id) {
    image_pyramid_[cam_id].resize(tracker_options_.pyramid_levels);
    images[cam_id].copyTo(image_pyramid_[cam_id][0]);
    //image_pyramid_[cam_id][0] = images[cam_id];
    std::cout << "There are: " << tracker_options_.pyramid_levels << " levels\n";
    for (uint32_t ii = 1 ; ii < tracker_options_.pyramid_levels ; ++ii) {
      std::cout << "Pyramid: " << ii-1 << " Size: " << image_pyramid_[cam_id][ii-1].rows << "x" << image_pyramid_[cam_id][ii-1].cols << "\n";
      cv::resize(image_pyramid_[cam_id][ii - 1], image_pyramid_[cam_id][ii], cv::Size(0, 0), 0.5, 0.5);
    }
  }

  for (uint32_t ii = 0 ; ii < tracker_options_.pyramid_levels ; ++ii) {
    pyramid_coord_ratio_[ii][0] =
        (double)(image_pyramid_[0][ii].cols) /
        (double)(image_pyramid_[0][0].cols);
    pyramid_coord_ratio_[ii][1] =
        (double)(image_pyramid_[0][ii].rows) /
        (double)(image_pyramid_[0][0].rows);
  }

  if (last_image_was_keyframe_) {
    uint32_t max_length = 0;
    for (std::shared_ptr<DenseTrack>& track : current_tracks_)
    {
      track->keypoints.emplace_back(num_cameras_);

      if (track->keypoints.size() > max_length) {
        longest_track_id_ = track->id;
        max_length = track->keypoints.size();
      }
    }
  }

  last_image_was_keyframe_ = false;
}
