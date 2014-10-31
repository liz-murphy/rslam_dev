#pragma once
#include <vector>
#include <list>
#include <Eigen/Eigen>
#include <opencv2/features2d/features2d.hpp>

namespace sdtrack
{
  struct Track;
  struct DenseTrack;

  // As an optimization, this structure can be templated on patch size and
  // number of pyrmaid levels to keep memory on the stack.
  struct Patch
  {
    Patch(uint32_t patch_dimension)
    {
      dim = patch_dimension;
      values.resize(patch_dimension * patch_dimension);
      rays.resize(values.size());
      dprojections.resize(values.size());
    }



    Eigen::Vector2d center;
    uint32_t dim;
    double mean;
    double projected_mean;
    std::vector<double> values;
    std::vector<Eigen::Matrix<double, 1, 6>> dprojections;
    std::vector<Eigen::Vector3d> rays;
    // These are always in the 0th level coordinates
    // std::vector<Eigen::Vector2d> projections;
    // std::vector<double> projected_values;
    std::vector<Eigen::Vector2i> indices;
    // std::vector<double> residuals;

  };


  struct DenseKeypoint
  {
    DenseKeypoint(uint32_t num_pyrmaid_levels,
                  const std::vector<uint32_t>& pyramid_dims)
    {
      patch_pyramid.reserve(num_pyrmaid_levels);
      for (size_t ii = 0 ; ii < num_pyrmaid_levels ; ++ii)
      {
        patch_pyramid.push_back(Patch(pyramid_dims[ii]));
      }
    }

    double rho = 1.0; // inverse depth
    double old_rho = 1.0;
    double response = 0;
    double response2 = 0;
    Eigen::Vector2d center_px;
    Eigen::Vector3d ray;
    DenseTrack* track = nullptr;
    double x;
    double y;
    std::vector<Patch> patch_pyramid;
  };

}
