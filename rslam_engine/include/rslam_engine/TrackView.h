#ifndef TRACK_VIEW_H
#define TRACK_VIEW_H

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

class KeypointsView
{
  public:
    std::vector<cv::KeyPoint> m_vKeypts;
    cv::Mat m_Image;
};

class MultiViewTrackInfo
{
  public:
    MultiViewTrackInfo(const calibu::CameraRigT<Scalar> &rig) {
      for(size_t ii=0; ii < rig.cameras.size(); ii++)
      {
        image_views.push_back(std::make_shared<KeypointsView>());
      }
    }

    void Update(const std::vector<cv::Mat>& vImages,
        const std::vector<MultiViewMeasurement>& vMeasurements,
        const std::vector<std::vector<cv::KeyPoint> >& vMultiKeypts,
        const ReferenceFrameId& frame_id_in)
    {
      frame_id = frame_id_in;
      measurements = vMeasurements;
      for(size_t ii=0; ii < vMultiKeypts.size(); ++ii)
      {
        image_views[ii]->m_vKeypts = vMultiKeypts[ii];
        image_views[ii]->m_Image = vImages[ii];
      }
    }

  ReferenceFrameId frame_id;
  std::vector<MultiViewMeasurement> measurements;
  std::vector<std::shared_ptr<KeypointsView> > image_views;

};

using MultiViewTrackInfoPtr = std::shared_ptr<MultiViewTrackInfo>;

class TrackView
{
  public:
    void InitReset(const std::shared_ptr<SlamMapProxy> &map,
        const calibu::CameraRigT<Scalar> &rig,
        unsigned int num_views = 2)
    {
      map_ = map;
      rig_ = rig;
      max_track_length = 16;

      // clean up existing data
      while(track_info_.size() >= 1)
        track_info_.pop_back();

    }

    void Update(const std::vector<cv::Mat>& vImages,
        const std::vector<MultiViewMeasurement>& vMeasurements,
        const std::vector<std::vector<cv::KeyPoint> >& vKeypts,
        const ReferenceFrameId& frame_id)
    {
      MultiViewTrackInfoPtr pMultiView;
      if(track_info_.size() < num_visible_time_steps_)
      {
        pMultiView = MultiViewTrackInfoPtr(new MultiViewTrackInfo(rig_));
      }
      else
      {
        pMultiView = track_info_.back();
        track_info_.pop_back();
      }
      track_info_.push_front(pMultiView);
      pMultiView->Update(vImages, vMeasurements, vKeypts, frame_id);
    }

    void Render(cv::Mat &left, cv::Mat &right)
    {
      // Draw keypoints on images and correspondence lines between both



    }

  private:
    std::shared_ptr<SlamMapProxy> map_;
    std::deque<std::shared_ptr<MultiViewTrackInfo>> track_info_;
    int max_track_length;
    calibu::CameraRigT<double> rig_;
    int num_visible_time_steps_;
};

#endif
