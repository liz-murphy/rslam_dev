#include <math.h>
#include <Accuracy.h>
#include <CartesianCoordinates.h>
#include <CoordinateConversionException.h>
#include <CoordinateConversionService.h>
#include <GeodeticCoordinates.h>
#include <GeodeticParameters.h>
#include <HeightType.h>
#include <LocalCartesianParameters.h>
#include <geocon/geodetic2local.h>
#include <miniglog/logging.h>
namespace geocon {

static MSP::CCS::GeodeticParameters kEllipsoidParameters(
    MSP::CCS::CoordinateType::geodetic,
    MSP::CCS::HeightType::ellipsoidHeight);

geodetic2local* geodetic2local::Create(double latitude,
                                       double longitude,
                                       double altitude) {
  MSP::CCS::LocalCartesianParameters localParameters(
      MSP::CCS::CoordinateType::localCartesian,
      // Longitude, latitude, altitude, orientation.
      longitude, latitude, altitude, 0);
  try {
    return new geodetic2local(
        new MSP::CCS::CoordinateConversionService("WGE", &kEllipsoidParameters,
                                                  "WGE", &localParameters));
  } catch (MSP::CCS::CoordinateConversionException& e) {
    LOG(ERROR) << "Failed to create ccs_: " << e.getMessage();
  }
  return nullptr;
}

geodetic2local::geodetic2local(MSP::CCS::CoordinateConversionService* ccs)
    : ccs_(ccs) {
}

bool geodetic2local::to_local(
    double latitude, double longitude, double altitude, double std,
    MSP::CCS::CartesianCoordinates* cart, MSP::CCS::Accuracy* acc) const {
  CHECK_NOTNULL(cart);
  CHECK_NOTNULL(acc);
  MSP::CCS::GeodeticCoordinates geo_coords(MSP::CCS::CoordinateType::geodetic,
                                           longitude, latitude, altitude);
  MSP::CCS::Accuracy geo_acc;

  // 90% confidence interval is 1.64 STD
  geo_acc.setSphericalError90(std * 1.64);

  try {
    ccs_->convertSourceToTarget(&geo_coords, &geo_acc, *cart, *acc);
  } catch (MSP::CCS::CoordinateConversionException& e) {
    LOG(ERROR) << "Failed to convert coordinates: " << e.getMessage();
    return false;
  }
  return true;
}

bool geodetic2local::to_geodetic(
    double x, double y, double z, double std,
    MSP::CCS::GeodeticCoordinates* geo, MSP::CCS::Accuracy* acc) const {
  CHECK(false);
  return false;
}
}  // namespace geocon
