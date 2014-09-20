project(geotrans)
cmake_minimum_required(VERSION 2.8)

# To get around Github's file size limits, we have to split this file into multiple pieces
set(LARGE_DATA_FILE Und_min2.5x2.5_egm2008_WGS84_TideFree_reformatted)
add_custom_command(OUTPUT data/${LARGE_DATA_FILE}
  COMMAND cat split/${LARGE_DATA_FILE}_* > ${LARGE_DATA_FILE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data
  )
add_custom_target(AssembleData ALL SOURCES data/${LARGE_DATA_FILE})

add_subdirectory(CCS)