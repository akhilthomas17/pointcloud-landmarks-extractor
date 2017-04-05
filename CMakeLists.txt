cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pole_detector)

find_package(PCL 1.3 REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS}
include
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

find_package(HDF5)
if(HDF5_FOUND)

  find_package(FLANN)
  include_directories(${FLANN_INCLUDE_DIRS})

  include_directories(${HDF5_INCLUDE_DIR})
  add_executable(pole_detector
    src/pole_detector.cpp
	src/main.cpp)

  target_link_libraries(pole_detector ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

  add_executable(build_tree src/build_tree.cpp)
  target_link_libraries(build_tree ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

endif(HDF5_FOUND)
