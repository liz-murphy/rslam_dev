#pragma once

#include <Accuracy.h>
#include <CartesianCoordinates.h>
#include <GeodeticCoordinates.h>
#include <CoordinateConversionService.h>
#include <memory>

namespace geocon {

/**
 * Convert easily between a local Cartesian coordinate system and
 * WGS84 Geodetic coordinates.
 */
class geodetic2local {
 public:
  virtual ~geodetic2local() {}

  /**
   * Create using initial geodetic coordinates in radians.
   *
   * Can return nullptr if initialization fails (possibly because of
   * missing data).
   */
  static geodetic2local* Create(double latitude,
                                double longitude,
                                double altitude);

  /**
   * Convert from WGS84 geodetic to local cartesian plane coordinates.
   *
   * @param latitude In radians
   * @param longitude In radians
   * @param altitude In meters. Techinically, Height Above Ellipsoid.
   * @param std Spherical standard deviation of position error in meters.
   */
  bool to_local(
      double latitude, double longitude, double altitude, double std,
      MSP::CCS::CartesianCoordinates* cart, MSP::CCS::Accuracy* acc) const;

  /**
   * Convert from the initialized local plane coordinates into WGS84
   * Geodetic coordinates
   */
  bool to_geodetic(
      double x, double y, double z, double std,
      MSP::CCS::GeodeticCoordinates* geo, MSP::CCS::Accuracy* acc) const;

 private:
  // Create using initial geodetic coordinates in radians
  geodetic2local(MSP::CCS::CoordinateConversionService* ccs);

  std::unique_ptr<MSP::CCS::CoordinateConversionService> ccs_;
};
}  // namespace geocon
