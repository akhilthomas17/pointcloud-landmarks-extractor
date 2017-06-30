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

  add_executable(landmark_classifier
          src/Master.cpp
          src/landmark_classifier.cpp
          src/Preprocessor/Preprocessor.cpp
          src/Segmenter/Segmenter.cpp
          src/Detector/RuleBasedDetector.cpp
          src/Classifier/FeatureExtractor.cpp
          src/Classifier/RandomForestLearner.cpp
          src/Classifier/FeatureBasedClassifier.cpp
          )

  target_link_libraries(landmark_classifier ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} ${OpenCV_LIBS})

endif(HDF5_FOUND)
