//
// Created by akhil on 03.05.17.
//

#ifndef RANDOMFORESTLEARNER_HPP
#define RANDOMFORESTLEARNER_HPP

#include <opencv2/ml/ml.hpp>
#include "Libs/Structures.hpp"
using namespace cv;

class RandomForestLearner {
private:
    CvRTrees* treeClassifier;
    CvRTrees* poleClassifier;
    CvRTrees* poleTreeClassifier;

public:
    RandomForestLearner();
    ~RandomForestLearner();
    void loadTrainingData(string pathToFolder, cv::Mat& trainFeatures, cv::Mat& trainLabels);
    void trainMultiClass(const cv::Mat &trainFeatures, const cv::Mat &trainLabels, CvRTParams trees_params);
    void saveClassifier(string pathToClassifier);
    void predictClass(Feature candidate, string& predictedClass);
    void loadClassifier(string pathToClassifier);
};


#endif //POLE_DETECTOR_RANDOMFORESTLEARNER_HPP
