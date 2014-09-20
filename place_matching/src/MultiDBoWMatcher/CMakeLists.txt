cmake_minimum_required(VERSION 2.8)

set( CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}" )
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}" )
endif()

set(PLACEMATCH_SRCS
  ${PLACEMATCH_SRCS}
  ${CMAKE_CURRENT_SOURCE_DIR}/MultiDBoWMatcher.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/AdjacencyWindow.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/AdjacencyWindowManager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Parameters.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/LandmarkVisitor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/WindowVisitor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/WindowTrack.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Session.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Persistence.cpp

  PARENT_SCOPE
)
