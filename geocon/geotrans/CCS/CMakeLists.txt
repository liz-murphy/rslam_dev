set(DTCC_SRCS
  threads/CCSThreadMutex.cpp 
  threads/CCSThreadLock.cpp 
  albers/AlbersEqualAreaConic.cpp 
  azeq/AzimuthalEquidistant.cpp 
  bng/BritishNationalGrid.cpp 
  bonne/Bonne.cpp 
  cassini/Cassini.cpp 
  cyleqa/CylindricalEqualArea.cpp 
  datum/Datum.cpp 
  datum/DatumLibraryImplementation.cpp 
  datum/SevenParameterDatum.cpp 
  datum/ThreeParameterDatum.cpp 
  eckert4/Eckert4.cpp 
  eckert6/Eckert6.cpp 
  ellipse/Ellipsoid.cpp 
  ellipse/EllipsoidLibraryImplementation.cpp 
  eqdcyl/EquidistantCylindrical.cpp 
  gars/GARS.cpp 
  geocent/Geocentric.cpp 
  georef/GEOREF.cpp 
  gnomonic/Gnomonic.cpp 
  grinten/VanDerGrinten.cpp 
  lambert/LambertConformalConic.cpp 
  loccart/LocalCartesian.cpp 
  mercator/Mercator.cpp 
  mgrs/MGRS.cpp 
  miller/MillerCylindrical.cpp 
  misc/CoordinateSystem.cpp 
  mollweid/Mollweide.cpp 
  neys/Neys.cpp 
  nzmg/NZMG.cpp 
  omerc/ObliqueMercator.cpp 
  orthogr/Orthographic.cpp 
  polarst/PolarStereographic.cpp 
  polycon/Polyconic.cpp 
  sinusoid/Sinusoidal.cpp 
  stereogr/Stereographic.cpp 
  trcyleqa/TransverseCylindricalEqualArea.cpp 
  tranmerc/TransverseMercator.cpp 
  ups/UPS.cpp 
  usng/USNG.cpp 
  utm/UTM.cpp
  )

# Given a variable name (not a list itself), prepend (in-place) the
# given prefix to every item in the list
function(prepend_list src_list prefix)
  set(tmp_list)
  foreach(src ${${src_list}})
    list(APPEND tmp_list ${prefix}/${src})
  endforeach()
  set(${src_list} ${tmp_list} PARENT_SCOPE)
endfunction()

prepend_list(DTCC_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/dtcc/CoordinateSystems/)

set(CCS_SRCS src/CoordinateConversion/CoordinateConversionService.cpp)

set(CCSERVICE_SRCS
  DatumLibrary.cpp 
  EllipsoidLibrary.cpp 
  GeoidLibrary.cpp 
  egm2008_geoid_grid.cpp 
  egm2008_full_grid_package.cpp 
  egm2008_aoi_grid_package.cpp 
  CoordinateSystemParameters/CoordinateSystemParameters.cpp 
  CoordinateSystemParameters/EllipsoidParameters.cpp 
  CoordinateSystemParameters/EquidistantCylindricalParameters.cpp 
  CoordinateSystemParameters/GeodeticParameters.cpp 
  CoordinateSystemParameters/LocalCartesianParameters.cpp 
  CoordinateSystemParameters/MapProjection3Parameters.cpp 
  CoordinateSystemParameters/MapProjection4Parameters.cpp 
  CoordinateSystemParameters/MapProjection5Parameters.cpp 
  CoordinateSystemParameters/MapProjection6Parameters.cpp 
  CoordinateSystemParameters/MercatorStandardParallelParameters.cpp 
  CoordinateSystemParameters/MercatorScaleFactorParameters.cpp 
  CoordinateSystemParameters/NeysParameters.cpp 
  CoordinateSystemParameters/ObliqueMercatorParameters.cpp 
  CoordinateSystemParameters/PolarStereographicStandardParallelParameters.cpp 
  CoordinateSystemParameters/PolarStereographicScaleFactorParameters.cpp 
  CoordinateSystemParameters/UTMParameters.cpp 
  CoordinateTuples/Accuracy.cpp 
  CoordinateTuples/BNGCoordinates.cpp 
  CoordinateTuples/CartesianCoordinates.cpp 
  CoordinateTuples/CoordinateTuple.cpp 
  CoordinateTuples/GARSCoordinates.cpp 
  CoordinateTuples/GeodeticCoordinates.cpp 
  CoordinateTuples/GEOREFCoordinates.cpp 
  CoordinateTuples/MapProjectionCoordinates.cpp 
  CoordinateTuples/MGRSorUSNGCoordinates.cpp 
  CoordinateTuples/UPSCoordinates.cpp 
  CoordinateTuples/UTMCoordinates.cpp 
  Exception/ErrorMessages.cpp 
  Exception/WarningMessages.cpp
  )
prepend_list(CCSERVICE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/dtcc)

set(DTCC_INC_DIRS
  threads 
  albers 
  azeq 
  bonne 
  bng 
  cassini 
  cyleqa 
  datum 
  eckert4 
  eckert6 
  ellipse 
  eqdcyl 
  gars 
  geocent 
  geoid 
  georef 
  gnomonic 
  grinten 
  lambert 
  loccart 
  mercator 
  misc 
  mgrs 
  miller 
  mollweid 
  neys 
  nzmg 
  omerc 
  orthogr 
  polarst 
  polycon 
  sinusoid 
  stereogr 
  trcyleqa 
  tranmerc 
  ups 
  usng 
  utm
  )
prepend_list(DTCC_INC_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/dtcc/CoordinateSystems)

set(CCS_INC_DIRS
  dtcc 
  CoordinateConversion 
  dtcc/CoordinateSystemParameters 
  dtcc/CoordinateTuples 
  dtcc/Enumerations 
  dtcc/Exception 
  )
prepend_list(CCS_INC_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(LIB_INC_DIR ${CCS_INC_DIRS} ${DTCC_INC_DIRS})
set(GEOTRANS_INCLUDE_DIRS ${LIB_INC_DIR} CACHE INTERNAL "")
set(GEOTRANS_LINK_DIRS ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
include_directories(${LIB_INC_DIR})

# Datum library
add_library(MSPdtcc
  ${DTCC_SRCS} ${CCSERVICE_SRCS}
  )

# Coordinate Conversion Service
add_library(msp_ccs
  ${CCS_SRCS}
  )

#export_package(GEOTRANS
#  TARGETS MSPdtcc msp_ccs
#  LIBRARIES MSPdtcc msp_ccs
#  INCLUDE_DIRS ${LIB_INC_DIR}
#  )
