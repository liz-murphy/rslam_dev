#include <place_matching/DBoWMatcher/DLoopDetector/TemplatedLoopDetector.h>

// --------------------------------------------------------------------------

DLoopDetector::Parameters::Parameters(float frequency):
  image_rows(0), image_cols(0),
  use_nss(true), alpha(0.3), k(3), geom_check(GEOM_DI), di_levels(2),
  geom_algorithm(GEOM_ALG_FUNDAMENTAL_MATRIX)
{
  set(frequency);
}

// --------------------------------------------------------------------------

DLoopDetector::Parameters::Parameters
  (int height, int width, float frequency, bool nss, float _alpha,
  int _k, GeometricalCheck geom, int dilevels, GeometricalAlgorithm alg)
  : image_rows(height), image_cols(width), use_nss(nss), alpha(_alpha), k(_k),
    geom_check(geom), di_levels(dilevels), geom_algorithm(alg)
{
  set(frequency);
}

// --------------------------------------------------------------------------

void DLoopDetector::Parameters::set(float f)
{
  dislocal = 20 * f;
  max_db_results = 50 * f;
  min_nss_factor = 0.005;
  min_matches_per_group = f;
  max_intragroup_gap = 3 * f;
  max_distance_between_groups = 3 * f;
  max_distance_between_queries = 2 * f;

  min_Fpoints = (geom_algorithm == GEOM_ALG_FUNDAMENTAL_MATRIX ? 12 : 8);
  max_ransac_iterations = 500;
  ransac_probability = 0.99;
  max_reprojection_error = 2.0;

  max_neighbor_ratio = 0.6;
}

// --------------------------------------------------------------------------
