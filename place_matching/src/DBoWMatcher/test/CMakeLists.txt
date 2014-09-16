project(DBoWMatcher-test)
cmake_minimum_required(VERSION 2.8)

include(def_executable)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
elseif (NOT ANDROID)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

find_package(HAL 0.1 REQUIRED)
find_package(OpenCV2 REQUIRED)
find_package(Calibu 0.1 REQUIRED)

include_directories(
  ${HAL_INCLUDE_DIRS}
  ${PlaceMatcher_BINARY_DIR}	# We need its protobuf files
  )

link_directories(${HAL_LINK_DIRS})

if(APPLE AND NOT BUILD_SHARED_LIBS)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,-all_load)
elseif(ANDROID)
  set(LINK_WHOLE_ARCHIVE_FLAG -Wl,--whole-archive)
  set(NO_LINK_WHOLE_ARCHIVE_FLAG -Wl,--no-whole-archive)
endif()

def_executable(DBoWMatcher-test
  SOURCES main.cpp LoopApp.cpp
  DEPENDS
  PlaceMatcher 
  SlamUtils
  miniglog

  LINK_LIBS
  ${LINK_WHOLE_ARCHIVE_FLAG}
  ${HAL_LIBRARIES}
  ${NO_LINK_WHOLE_ARCHIVE_FLAG}

  ${CVARS_LIBRARY}
  ${OpenCV2_LIBRARIES}
  dl
  )


