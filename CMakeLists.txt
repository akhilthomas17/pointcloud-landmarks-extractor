cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pole_detector)

find_package(PCL 1.3 REQUIRED)
find_package( OpenCV REQUIRED )


include_directories(
        ${OpenCV_INCLUDE_DIRS}
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
	src/main.cpp
	src/FeatureDescriptor.cpp
          src/RandomForestLearner.cpp)

  target_link_libraries(pole_detector ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} ${OpenCV_LIBS})

  add_executable(train_knn
          src/train_knn.cpp
          src/FeatureDescriptor.cpp)

  target_link_libraries(train_knn ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})
  add_executable(train_randomForest
          src/train_randomForest.cpp
          src/RandomForestLearner.cpp
          src/FeatureDescriptor.cpp)

  target_link_libraries(train_randomForest ${PCL_LIBRARIES} ${Boost_LIBRARIES}
          ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} ${OpenCV_LIBS})

endif(HDF5_FOUND)
