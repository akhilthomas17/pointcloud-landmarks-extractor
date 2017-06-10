//
// Created by akhil on 10.06.17.
//

#ifndef FEATUREBASEDCLASSIFIER_HPP
#define FEATUREBASEDCLASSIFIER_HPP


#include <Libs/Structures.hpp>
#include <Classifier/FeatureExtractor.hpp>
#include <Classifier/RandomForestLearner.hpp>
#include <flann/io/hdf5.h>

class FeatureBasedClassifier {
public:
    FeatureBasedClassifier(list<Segment>* poleLikeObjects_, list<Segment>* detectedPoles_,
                           list<Segment>* detectedTrees_):
            poleLikeObjects(poleLikeObjects_),
            detectedPoles(detectedPoles_),
            detectedTrees(detectedTrees_){}
    void loadRandomForest(string pathToClassifier);
    void loadKnn(string pathToKnn, double knnThreshold_);
    void featureMatcher(list<Segment> stitchedClusters, int mode, bool rForest);


private:
    list<Segment>* poleLikeObjects;
    list<Segment>* detectedPoles;
    list<Segment>* detectedTrees;
    RandomForestLearner randomForest;
    flann::Index<flann::ChiSquareDistance<float> > * kdTree;
    vector<string> knnLabelList;
    double knnThreshold;

    void loadKnnLabels(string name_file);
    void predictClassKnn(Feature feature, string& predictedClass);

};


#endif //POLE_DETECTOR_FEATUREBASEDCLASSIFIER_HPP
