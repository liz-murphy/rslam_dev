cmake_minimum_required(VERSION 2.8.3)
project(place_matching)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  slam_map
  utils
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

set(PLACEMATCH_SRCS src/PlaceMatcher.cpp src/PlaceMatcherFactory.cpp src/TemplateMatcher/TemplateMatcher.cpp)

set(TEMPLATE_MATCHER_SRCS src/TemplateMatcher/TemplateMatcher.cpp)

set(DBOW_SRCS 
  src/DBoWMatcher/DBoWMatcher.cpp
  src/DBoWMatcher/FMat32F.cpp
  src/DBoWMatcher/FMat8UBinary.cpp
  src/DBoWMatcher/DUtils/BinaryFile.cpp
  src/DBoWMatcher/DUtils/ConfigFile.cpp
  src/DBoWMatcher/DUtils/DebugFunctions.cpp
  src/DBoWMatcher/DUtils/FileFunctions.cpp
  src/DBoWMatcher/DUtils/LineFile.cpp
  src/DBoWMatcher/DUtils/LUT.cpp
  src/DBoWMatcher/DUtils/Profiler.cpp
  src/DBoWMatcher/DUtils/Random.cpp
  src/DBoWMatcher/DUtils/StringFunctions.cpp
  src/DBoWMatcher/DUtils/TimeManager.cpp
  src/DBoWMatcher/DUtils/Timestamp.cpp

  src/DBoWMatcher/DUtilsCV/Drawing.cpp
  src/DBoWMatcher/DUtilsCV/Geometry.cpp
  src/DBoWMatcher/DUtilsCV/GUI.cpp
  src/DBoWMatcher/DUtilsCV/IO.cpp
  src/DBoWMatcher/DUtilsCV/Mat.cpp
  src/DBoWMatcher/DUtilsCV/Transformations.cpp
  src/DBoWMatcher/DUtilsCV/Types.cpp

  src/DBoWMatcher/DVision/BundleCamera.cpp
  src/DBoWMatcher/DVision/FSolver.cpp
  src/DBoWMatcher/DVision/HSolver.cpp
  src/DBoWMatcher/DVision/ImageFunctions.cpp
  src/DBoWMatcher/DVision/Matches.cpp
  src/DBoWMatcher/DVision/PatchFile.cpp
  src/DBoWMatcher/DVision/PixelPointFile.cpp
  src/DBoWMatcher/DVision/PLYFile.cpp
  src/DBoWMatcher/DVision/PMVSCamera.cpp
  src/DBoWMatcher/DVision/SurfSet.cpp

  src/DBoWMatcher/DBoW2/BowVector.cpp
  src/DBoWMatcher/DBoW2/FeatureVector.cpp
  src/DBoWMatcher/DBoW2/FSurf64.cpp
  src/DBoWMatcher/DBoW2/QueryResults.cpp
  src/DBoWMatcher/DBoW2/ScoringObject.cpp
  src/DBoWMatcher/DLoopDetector/TemplatedLoopDetector.cpp
)

configure_file(
  src/DBoWMatcher/dbow_config.h.in
  include/place_matching/DBoWMatcher/dbow_config.h
)

set(MULTI_DBOW_SRCS
  src/MultiDBoWMatcher/MultiDBoWMatcher.cpp
  src/MultiDBoWMatcher/AdjacencyWindow.cpp
  src/MultiDBoWMatcher/AdjacencyWindowManager.cpp
  src/MultiDBoWMatcher/Parameters.cpp
  src/MultiDBoWMatcher/LandmarkVisitor.cpp
  src/MultiDBoWMatcher/WindowVisitor.cpp
  src/MultiDBoWMatcher/WindowTrack.cpp
  src/MultiDBoWMatcher/Session.cpp
  src/MultiDBoWMatcher/Node.cpp
  src/MultiDBoWMatcher/Persistence.cpp
)

option(DOWNLOAD_DBoWMatcher_ORB_Voc
  "Download ORB vocabulary for DBoWMatcher loop detector"
  ON)

set(DBOW_ORB_VOC_FILE vocabularies/ORB_k10L5.voc.gz)
if(DOWNLOAD_DBoWMatcher_ORB_Voc)
file(DOWNLOAD
  http://arpg.colorado.edu/Internal/resources/DBoWMatcher_vocs/ORB_k10L5.voc.gz
  ${DBOW_ORB_VOC_FILE}
  EXPECTED_MD5 b6d561716dc3799eb594026986f68020
  SHOW_PROGRESS)

#add_subdirectory(src/MultiDBoWMatcher)

#configure_file ("${PROJECT_SOURCE_DIR}/include/place_matching/DBoWMatcher/dbow_config.h.in"
#  "${PROJECT_SOURCE_DIR}/include/place_matching/DBoWMatcher/dbow_config.h" )

#option(BUILD_DBoWMatcher "Build DBoWMatcher loop detector" OFF)
#if(BUILD_DBoWMatcher)
#    add_subdirectory(src/DBoWMatcher/DBoWMatcher)      # DBM_ vars
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS} 
)

add_library(place_matching
  ${PLACEMATCH_SRCS}
  ${TEMPLATE_MATCHER_SRCS}
  ${DBOW_SRCS}
  ${MULTI_DBOW_SRCS}
)

target_link_libraries(place_matching ${OpenCV_LIBRARIES})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES place_matching
  CATKIN_DEPENDS roscpp  
  DEPENDS ${OpenCV2_LIBRARIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(place_matching
#   src/DBoWMatcher/${PROJECT_NAME}/place_matching.cpp
# )

## Declare a cpp executable
# add_executable(place_matching_node src/DBoWMatcher/place_matching_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(place_matching_node place_matching_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(place_matching_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS place_matching 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_place_matching.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
