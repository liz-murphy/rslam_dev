// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <Utils/GetPot>
#include <miniglog/logging.h>
#include <PlaceMatching/DBoWMatcher/DBoWMatcher.h>
#include <PlaceMatching/DBoWMatcher/DUtilsCV/DUtilsCV.h>
#include "LoopApp.h"

const char* g_usage =
    "USAGE: LoopDetector-test <options>\n"
    "DBoWMatcher test application"
    "\n"
    "\t--dbow_voc_file\tPath to vocabulary file\n"
    "\t--frequency\tFrequency at which the images are processed by the detector\n"
    "\n"
    "Examples: \n\n"
    " LoopDetector-test -cam rectify:[file=cameras.xml]//deinterlace://uvc:// "
    "--dbow_voc_file /path/ORB_k10L5.voc.gz --frequency 3\n"
    " LoopDetector-test -cam file:[loop=1]//~/Data/CityBlock-Noisy/[left*,right*].pgm "
    "--dbow_voc_file /path/ORB_k10L5.voc.gz --frequency 3\n";

LoopApp g_app;
DBoWMatcher g_matcher;

void run();
cv::Mat getMatchImage(const cv::Mat &left, const cv::Mat &right = cv::Mat());

int main(int argc, char** argv)
{
  GetPot cl(argc, argv);
  if (cl.search("--help")) {
    LOG(INFO) << g_usage;
    exit(-1);
  }

  try
  {
    // load cameras
    g_app.Init(cl);

    // get an image to get its dimensions
    cv::Mat im = g_app.getImage();

    if(!im.empty())
    {
      std::string voc_file = cl.follow("", "--dbow_voc_file");
      double frequency = cl.follow(1, "--frequency");

      LOG(INFO) << "Loading vocabulary...";

      // g_matcher.setVocabulary(voc_file) uses default parameters
      // that we can override:
      //   use_nss = true; // use normalized similarity score instead of raw one
      //   alpha = 0.3; // acceptance nss threshold
      //   k = 3; // temporal consistency
      //   geom_check = DLoopDetector::GEOM_DI; // use direct index for geom
      //   di_levels = 2; // use two direct index levels
      //   geom_algorithm = DLoopDetector::GEOM_ALG_FUNDAMENTAL_MATRIX;
      //   min_Fpoints = 12; // number of inliers to accept a fundamental matrix
      //     (or a homography)
      //
      // The function Parameters::set(frequency) sets some parameters that
      // are better defined according to the working frequency on the sequence.
      // For example:
      // dislocal = 20*f by default: number of recent images that are ignored
      // max_db_results = 50*f by default: max number of places
      //   to retrieve from the database

      DBoWMatcher::Parameters params(frequency);

      // we can use a homography instead of a fundamental matrix
      params.geom_algorithm = DLoopDetector::GEOM_ALG_HOMOGRAPHY;

      g_matcher.setVocabulary(voc_file, params);

      // ok, lets go
      run();
    }
  }
  catch(std::string &ex)
  {
    LOG(INFO) << "Error: " << ex;
  }
  catch(std::exception &ex)
  {
    LOG(INFO) << "Error: " << ex.what();
  }

  LOG(INFO) << "Done";

  return 0;
}

void run()
{
  LOG(INFO) << "Running";

  DUtilsCV::GUI::tWinHandler win = "LoopDetector-test";
  DUtilsCV::GUI::tWinHandler winplot = "LoopDetector-test Trajectory";

  double minx, maxx, miny, maxy;
  g_app.getGroundTruthBounds(minx, maxx, miny, maxy);
  DUtilsCV::Drawing::Plot implot(240, 320, minx, maxx, -maxy, -miny, 20);

  const DUtilsCV::Drawing::Plot::Style normal_style(2); // thickness
  const DUtilsCV::Drawing::Plot::Style loop_style('r', 2); // color, thickness

  // these are just used for visualization
  std::vector<cv::Mat> all_places;

  // to show the matches
  cv::Mat match_im;
  bool frame_to_frame = false;

  cv::Mat im;
  double ts, prevx, prevy;
  im = g_app.getImage(ts, prevx, prevy);

  unsigned int new_place_id = 0;

  for(size_t frame_idx = 0; ; ++frame_idx)
  {
    double x, y;
    im = g_app.getImage(ts, x, y);
    if(im.empty()) break;

    std::vector<PlaceMatchCandidate> matches;

    //g_matcher.GetPotentialPlaceMatches(new_place_id, im, matches);
    //if(matches.empty()) g_matcher.AddPlace(new_place_id, im);

    // equivalent, but extracting features only once
    g_matcher.GetPotentialMatchesOrAddPlace(im, matches, new_place_id);

    if(!matches.empty())
    {
      LOG(INFO) << "Loop detected with frame id " << matches[0].getID();

      // visualization stuff
      match_im = getMatchImage(im, all_places[matches[0].getID()]);
    }
    else
    {
      ++new_place_id;
      // visualization stuff
      all_places.push_back(im.clone());
      match_im = getMatchImage(im);
    }

    // show match
    int key = DUtilsCV::GUI::showImage(match_im, true, &win,
                                   (frame_to_frame ? 0 : 10));

    if(key == DUtilsCV::GUI::ESC) break;
    else if(key == ' ') frame_to_frame = !frame_to_frame;

    // show the trajectory if provided
    if(ts > 0)
    {
      if(matches.empty())
        implot.line(prevx, -prevy, x, -y, normal_style);
      else
        implot.line(prevx, -prevy, x, -y, loop_style);

      prevx = x;
      prevy = y;

      DUtilsCV::GUI::showImage(implot.getImage(), true, &winplot, 10);
    }

    // skip some images (for newcollege or bicocca datasets)
    //for(int i = 0; i < 10; i++) g_app.getImage();
  }
}

cv::Mat getMatchImage(const cv::Mat &left, const cv::Mat &right)
{
  int W, H;

  if(right.empty())
  {
    W = left.cols * 2;
    H = left.rows;
  }
  else
  {
    W = left.cols + right.cols;
    H = std::max(left.rows, right.rows);
  }

  cv::Mat ret = cv::Mat::zeros(H, W, CV_8U);
  left.copyTo(ret( cv::Range(0, left.rows), cv::Range(0, left.cols) ));

  if(!right.empty())
  {
    right.copyTo(ret( cv::Range(0, right.rows), cv::Range(left.cols, W) ));
  }

  return ret;
}
