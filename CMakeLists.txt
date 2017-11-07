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

add_executable(approach_landa
        src/landa_approach.cpp
        src/Preprocessor/Preprocessor.cpp
        src/Segmenter/Segmenter.cpp
        src/Detector/RuleBasedDetector.cpp
        )
target_link_libraries(approach_landa
        ${PCL_LIBRARIES} ${Boost_LIBRARIES})


find_package(HDF5)
if(HDF5_FOUND)

  find_package(FLANN)
  include_directories(${FLANN_INCLUDE_DIRS})

  include_directories(${HDF5_INCLUDE_DIR})

  add_executable(pole_detector
          src/Master.cpp src/pole_detector.cpp
          src/Preprocessor/Preprocessor.cpp
          src/Segmenter/Segmenter.cpp
          src/Detector/RuleBasedDetector.cpp
          src/Classifier/FeatureExtractor.cpp
          src/Classifier/RandomForestLearner.cpp
          src/Classifier/FeatureBasedClassifier.cpp
          )

  target_link_libraries(pole_detector ${PCL_LIBRARIES} ${Boost_LIBRARIES}
          ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} ${OpenCV_LIBS})

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

  add_executable(train_knn
          src/train_knn.cpp
          src/Classifier/FeatureExtractor.cpp)

  target_link_libraries(train_knn ${PCL_LIBRARIES} ${Boost_LIBRARIES}
                                 ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})
  add_executable(train_randomForest
          src/train_randomForest.cpp
          src/Classifier/RandomForestLearner.cpp
          src/Classifier/FeatureExtractor.cpp)

  target_link_libraries(train_randomForest ${PCL_LIBRARIES} ${Boost_LIBRARIES}
          ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} ${OpenCV_LIBS})

endif(HDF5_FOUND)
