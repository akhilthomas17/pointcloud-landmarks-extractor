//
// Created by akhil on 03.05.17.
//

#ifndef POLE_DETECTOR_RANDOMFORESTLEARNER_HPP
#define POLE_DETECTOR_RANDOMFORESTLEARNER_HPP

#include <opencv2/ml/ml.hpp>
#include "common.hpp"

class RandomForestLearner {
private:
    CvRTrees* treeClassifier;
    CvRTrees* poleClassifier;
    CvRTrees* poleTreeClassifier;

public:
    RandomForestLearner();
    ~RandomForestLearner();
    void loadTrainingData(string pathToFolder, cv::Mat& trainFeatures, cv::Mat& trainLabels);
    void trainMultiClass(const cv::Mat& trainFeatures, const cv::Mat& trainLabels, int numTrees);
    void predictClass(Feature candidate, float maxDistance, string predictedClass);
    void loadClassifier(string pathToClassifier);
};


#endif //POLE_DETECTOR_RANDOMFORESTLEARNER_HPP
