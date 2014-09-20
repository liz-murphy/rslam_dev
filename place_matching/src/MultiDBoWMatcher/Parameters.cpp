#include <opencv2/opencv.hpp>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>


MultiDBoWMatcher::Parameters::Parameters():
  use_nss(true), alpha(0.3),
  k(3),
  use_di(false), di_level(4),
  dislocal(10.), // seconds
  max_db_results(100), // this parameter is usually set to 50 * frequency
  min_nss_factor(0.005),
  max_neighbor_ratio(0.8),
  geom_algorithm(PNP),
  min_Fpoints(12),
  max_reprojection_error(2.), // px
  ransac_probability(0.999),
  max_dist_window_nodes(3.), // metres (if map scaled)
  max_dist_matching_windows(3.), // metres (if map scaled)
  image_normalization(NO_NORMALIZATION)
{
}

void MultiDBoWMatcher::Parameters::save(cv::FileStorage &fs) const
{
  fs << "params" << "[:" <<
        (use_nss ? 1 : 0) <<
        alpha <<
        k <<
        (use_di ? 1 : 0) <<
        di_level <<
        dislocal <<
        max_db_results <<
        min_nss_factor <<
        max_neighbor_ratio <<
        static_cast<int>(geom_algorithm) <<
        min_Fpoints <<
        max_reprojection_error <<
        ransac_probability <<
        max_dist_window_nodes <<
        max_dist_matching_windows <<
        static_cast<int>(image_normalization) <<
        "]";
}

void MultiDBoWMatcher::Parameters::load(const cv::FileNode& fn)
{
  switch(fn.size())
  {
    case 16:
      image_normalization = static_cast<IMAGE_NORMALIZATION>
          (static_cast<int>(fn[15]));
    case 15:
      max_dist_matching_windows = static_cast<double>(fn[14]);
    case 14:
      max_dist_window_nodes = static_cast<double>(fn[13]);
    case 13:
      ransac_probability = static_cast<double>(fn[12]);
    case 12:
      max_reprojection_error = static_cast<double>(fn[11]);
    case 11:
      min_Fpoints = static_cast<int>(fn[10]);
    case 10:
      geom_algorithm =  static_cast<GEOM_ALGORITHM>(static_cast<int>(fn[9]));
    case 9:
      max_neighbor_ratio = static_cast<double>(fn[8]);
    case 8:
      min_nss_factor = static_cast<double>(fn[7]);
    case 7:
      max_db_results = static_cast<int>(fn[6]);
    case 6:
      dislocal = static_cast<double>(fn[5]);
    case 5:
      di_level = static_cast<int>(fn[4]);
    case 4:
      use_di = static_cast<int>(fn[3]) != 0;
    case 3:
      k = static_cast<int>(fn[2]);
    case 2:
      alpha = static_cast<double>(fn[1]);
    case 1:
      use_nss = static_cast<int>(fn[0]) != 0;
  }
}
