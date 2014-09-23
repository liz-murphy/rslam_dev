cmake_minimum_required(VERSION 2.8)

message("COMPILING DBoW...........................")
set( CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}" )
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}" )
endif()

set(DBM_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/DUtils
    ${CMAKE_CURRENT_SOURCE_DIR}/DUtilsCV
    ${CMAKE_CURRENT_SOURCE_DIR}/DVision
    ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2
    ${CMAKE_CURRENT_SOURCE_DIR}/DLoopDetector
    PARENT_SCOPE)

set(PLACEMATCH_SRCS 
  ${PLACEMATCH_SRCS}
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoWMatcher.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/FMat32F.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/FMat8UBinary.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/BinaryFile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/ConfigFile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/DebugFunctions.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/FileFunctions.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/LineFile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/LUT.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/Profiler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/Random.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/StringFunctions.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/TimeManager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/Timestamp.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/DUtilsCV/Drawing.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtilsCV/Geometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtilsCV/GUI.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtilsCV/IO.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtilsCV/Mat.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtilsCV/Transformations.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtilsCV/Types.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/BundleCamera.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/FSolver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/HSolver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/ImageFunctions.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/Matches.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/PatchFile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/PixelPointFile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/PLYFile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/PMVSCamera.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DVision/SurfSet.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/BowVector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/FeatureVector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/FSurf64.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/QueryResults.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/ScoringObject.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DLoopDetector/TemplatedLoopDetector.cpp

  PARENT_SCOPE
  )

# Download vocabularies

option(DOWNLOAD_DBoWMatcher_ORB_Voc
  "Download ORB vocabulary for DBoWMatcher loop detector"
  ON)

set(DBOW_ORB_VOC_FILE ${CMAKE_CURRENT_SOURCE_DIR}/vocabularies/ORB_k10L5.voc.gz)
if(DOWNLOAD_DBoWMatcher_ORB_Voc)
  file(DOWNLOAD
    http://rpg.robotics.gwu.edu/Internal/resources/DBoWMatcher_vocs/ORB_k10L5.voc.gz
    ${DBOW_ORB_VOC_FILE}
    #EXPECTED_MD5 b6d561716dc3799eb594026986f68020
    SHOW_PROGRESS)

  # Problem of ExternalProject_add: it requires a tar.gz and the content
  # of SOURCE_DIR is deleted first. DOWNLOAD_DIR can also be used, but then
  # the .tar.gz is not uncompressed.
  #ExternalProject_add(DBoWMatcher-ORB-voc
  #  URL http://rpg.robotics.gwu.edu/Internal/resources/DBoWMatcher_vocs/ORB_k10L5.voc.gz
  #  URL_MD5 b6d561716dc3799eb594026986f68020
  #  CONFIGURE_COMMAND ""
  #  BUILD_COMMAND ""
  #  INSTALL_COMMAND ""
  #  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vocabularies/
  #)

  #add_dependencies(PlaceMatcher DBoWMatcher-ORB-voc)
endif()

configure_file(
 ${CMAKE_CURRENT_SOURCE_DIR}/dbow_config.h.in
 ${CMAKE_CURRENT_SOURCE_DIR}/../../../include/place_matching/DBoWMatcher/dbow_config.h
  @ONLY
  )
