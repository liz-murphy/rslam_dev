cmake_minimum_required(VERSION 2.8)

set( CMAKE_CXX_FLAGS "-std=c++0x -Wall ${CMAKE_CXX_FLAGS}" )
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "-stdlib=libc++ ${CMAKE_CXX_FLAGS}" )
endif()

set(PLACEMATCH_SRCS 
  ${PLACEMATCH_SRCS}
  ${CMAKE_CURRENT_SOURCE_DIR}/TemplateMatcher.h
  ${CMAKE_CURRENT_SOURCE_DIR}/TemplateMatcher.cpp
  PARENT_SCOPE
  )

