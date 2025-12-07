cmake_minimum_required(VERSION 3.5)
project(fast_lio CXX)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
add_compile_options(-fdeclspec)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)

set(FASTLIO_MAPPING_SOURCES
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
  src/conversions.cpp
)

add_library(fastlio_mapping STATIC ${FASTLIO_MAPPING_SOURCES})
add_library(fastlio_mapping_shared SHARED ${FASTLIO_MAPPING_SOURCES})

set_target_properties(fastlio_mapping PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(fastlio_mapping_shared PROPERTIES POSITION_INDEPENDENT_CODE ON OUTPUT_NAME fastlio_mapping)

target_include_directories(fastlio_mapping PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
)
target_include_directories(fastlio_mapping_shared PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio_mapping
    PUBLIC OpenMP::OpenMP_CXX
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
  )
  target_link_libraries(fastlio_mapping_shared
    PUBLIC OpenMP::OpenMP_CXX
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
  )
else()
  target_link_libraries(fastlio_mapping
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
  )
  target_link_libraries(fastlio_mapping_shared
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_KDTREE_LIBRARIES}
  )
endif()
