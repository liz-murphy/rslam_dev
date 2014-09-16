#ifndef LOOPAPP_H
#define LOOPAPP_H

#include <Utils/GetPot>
#include <PlaceMatching/DBoWMatcher/DBoWMatcher.h>
#include <HAL/Camera/CameraDevice.h>
#include <Utils/ImageProcessing.h>
#include <calibu/Calibu.h>
#include <Utils/MathTypes.h>


class LoopApp
{
public:
  LoopApp();

  /**
   * @brief Init Initializes cameras
   * @param cl
   */
  void Init(GetPot &cl);

  /**
   * @brief getImage Returns the next image, if any, in an OpenCV format
   * @return im image or empty
   */
  cv::Mat getImage();

  /**
   * @brief getImage Returns the next image, if any, and its gt position
   * @param ts[out] timestamp, or -1
   * @param x[out] x
   * @param y[out] y
   * @return im image or empty
   */
  cv::Mat getImage(double &ts, double &x, double &y);

  /**
   * @brief getGroundTruthBounds If gt is available, this returns the bounds
   * of the x and y positions
   * @param minx[out]
   * @param maxx[out]
   * @param miny[out]
   * @param maxy[out]
   */
  void getGroundTruthBounds(double &minx, double &maxx, double &miny,
                            double &maxy) const;

protected:

  // timestamp -> <x,y>
  struct GroundTruth: std::map<double, std::pair<double, double>>
  {
    double minx, maxx, miny, maxy;

    void add(double ts, double x, double y);
  };

protected:
  bool InitResetCameras(GetPot &cl);
  bool Capture();
  bool HasLookupTable( const std::string& sRigFile );
  void LoadCurrentImages();
  void LoadTimestamps(const std::string &filename);
  void LoadGroundTruth(const std::string &filename);

protected:
  DBoWMatcher   matcher_;
  hal::Camera   camera_device_;
  calibu::CameraRigT<Scalar>   rig_;
  std::vector<Eigen::Vector4i>   rois_;
  std::shared_ptr<pb::ImageArray> images_;
  std::vector<double> timestamps_;
  ImageProcessing image_proc_;
  GroundTruth gt_;

  unsigned int start_frame_;
  bool is_rectified_;
  bool is_mono_tracking_;
  int active_camera_id_;
  bool is_using_sim_data_;

};

#endif // LOOPAPP_H
