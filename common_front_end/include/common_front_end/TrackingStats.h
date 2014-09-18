#pragma once

#include <map>
#include <vector>
#include <SlamMap/Measurement.h>

namespace rslam {
namespace common {

struct TrackingStats {
  unsigned int                            rig_size;
  std::map<std::string,float>             analytics;
  std::vector<std::map<MatchFlag,float> > match_flags_histogram;
  Sophus::SE3t                            t_wp;

  TrackingStats() : rig_size(2) {}

  void SetRigSize(unsigned int size) {
    rig_size=size;
  }

  void UpdateMeasurementHistograms(
      const std::vector<MultiViewMeasurement>& vNewMeasurements) {
    for (std::map<MatchFlag,float>& m : match_flags_histogram) {
      for (auto& pair : m) {
        pair.second = 0;
      }
    }

    if (vNewMeasurements.empty()) return;
    for (const MultiViewMeasurement& z : vNewMeasurements) {
      for (unsigned int nCam = 0; nCam < rig_size; nCam++) {
        match_flags_histogram[nCam][z.Flag(nCam)]++;
      }
    }
    int nTotal = 0;
    for (const std::map<MatchFlag,float>& m: match_flags_histogram) {
      for (const std::pair<MatchFlag,float>& pair : m) {
        nTotal += pair.second;
      }
    }

    for (std::map<MatchFlag,float>& m: match_flags_histogram) {
      for (std::pair<const MatchFlag,float>& pair : m) {
        pair.second /= nTotal;
      }
    }
  }

  void PrintMeasurementHistograms() {
    int nCam = 0;
    for (const std::map<MatchFlag,float>& m: match_flags_histogram) {
      printf("Cam[%d]:\n", nCam++);
      for (const std::pair<MatchFlag,float>& pair : m) {
        printf("  %17s: ", MatchStr(pair.first));
        for (int p = 0; p < 100*pair.second; p++) {
          printf("%c", '|');
        }
        printf("\n");
      }
    }
    fflush(stdout);
  }
};

}
}
