cmake_minimum_required(VERSION 2.8.3)
project(place_matching)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  slam_map
  utils
)
find_package(OpenCV REQUIRED)

add_subdirectory(src/TemplateMatcher)

option(BUILD_DBoWMatcher "Build DBoWMatcher loop detector" ON)
if(BUILD_DBoWMatcher)
  add_subdirectory(src/DBoWMatcher)      # DBM_ vars
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS} 
)

add_library(place_matching
  src/PlaceMatcher.cpp
  src/PlaceMatcherFactory.cpp
)

target_link_libraries(place_matching ${OpenCV2_LIBRARIES})

