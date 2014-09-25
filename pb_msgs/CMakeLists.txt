cmake_minimum_required(VERSION 2.8.3)
project(pb_msgs)

set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall  -Werror")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  sophus
  miniglog
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)

set( PROTO_IN
    # aux
    src/Matrix.proto 
   
    # header files
    src/Header.proto
    src/CameraModel.proto

    # message files
    src/Messages.proto
    src/Camera.proto
    src/Image.proto
    src/Imu.proto
    src/Pose.proto
    src/Encoder.proto
    src/Lidar.proto
    src/NodeCamMessage.proto
    src/NodeCar.proto
    src/CarPlanner.proto
    src/SceneGraphShapes.proto
    src/SimMessages.proto
    src/FrameObject.proto
    src/rslam.proto
    src/template.proto
    src/frontend.proto
    src/dbow.proto
    src/SlamServer.proto
)

set(arguments
  PROTOROOT ${PROJECT_SOURCE_DIR}/src
  PROTOFILES ${PROTO_IN}
  OUTPATH ${PROJECT_SOURCE_DIR}/include/pb_msgs 
  DEBUG
  )
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Eigen_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

configure_file(
  "${PROJECT_SOURCE_DIR}/include/pb_msgs/config.h.in"
  "${PROJECT_SOURCE_DIR}/include/pb_msgs/config.h"
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${arguments})

set(SRCS
    ${PROTO_SRCS}
    src/Logger.cpp
    src/Reader.cpp
    src/Image.cpp
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
  LIBRARIES pb_msgs
  CATKIN_DEPENDS roscpp sophus cmake_modules
  DEPENDS PROTOBUF 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
add_library(pb_msgs
  ${SRCS}
)

## Declare a cpp executable
# add_executable(pb_msgs_node src/pb_msgs_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pb_msgs_node pb_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pb_msgs
  ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES}
)

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
 install(TARGETS pb_msgs 
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pb_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
