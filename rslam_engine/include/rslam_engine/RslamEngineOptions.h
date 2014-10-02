// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <string>
#include <place_matching/PlaceMatcherFactory.h>

namespace rslam {

enum RslamTracker
{
  Tracker_Sparse = 0,
  Tracker_SemiDense = 1
};

struct RslamEngineOptions {
  // Is inertial data available
  bool imu = false;

  // Is position data available
  bool posys = false;

  // Use simulation data
  bool simulation = false;

  // Simulate the IMU
  bool simulation_imu = false;

  // Simulate a monocular camera
  bool simulation_mono = false;

  // Directory where simulation data is located
  std::string simulation_dir = "";

  // Feature detector to use
  std::string detector_type = "FAST";

  // Feature descriptor to use
  std::string descriptor_type = "PATCH";

  // Should a persistent backend be used for the map
  bool persistent_map = true;

  // Should the map be limited to a single track?
  bool single_track = false;

  // The name of the map file (used if persistent)
  std::string map_file = "SlamMap.db";

  // Should Rslam try to pick up where the last run left off.
  bool continue_run = false;

  // Should Rslam connect to a server
  bool use_server = false;

  // Type of server for Rslam to connect to.
  std::string server_type = "node";

  // Output filename for Rslam results
  std::string results_output = "rslam_results";

  // The working directory for Rslam. Programm will chdir here.
  std::string working_dir = "";

  // Options for loop closure detection
  PlaceMatcherFactory::Options place_matcher_options;

  RslamTracker tracker_type_ = Tracker_Sparse;
};
}  // namespace rslam
