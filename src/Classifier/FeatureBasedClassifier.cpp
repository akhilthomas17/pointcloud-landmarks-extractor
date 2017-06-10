//
// Created by akhil on 10.06.17.
//

#include "Classifier/FeatureBasedClassifier.hpp"

void FeatureBasedClassifier::featureMatcher(list<Segment> stitchedClusters, int featureMode, bool rForest) {
    for (list<Segment>::iterator it = stitchedClusters.begin(); it != stitchedClusters.end(); ++it){
        Segment candidate = *it;
        FeatureExtractor featureExtractor;

        /// Cluster filters based on "minimum height" and "maximum xy-bound"
        double minHeight = 2;
        double xyBoundThreshold = 20;
        if(!featureExtractor.filterSegment(&candidate, minHeight, xyBoundThreshold)){
            poleLikeObjects->push_back(candidate);
            /// Extracting the features as per the mode chosen
            Feature feature;
            switch (featureMode){
                case 0:
                    featureExtractor.extractEsfFeature(candidate.getSegmentCloud(), &feature);
                    break;
                case 1:
                    featureExtractor.extractEigenFeature(&candidate, &feature);
                    break;
                case 2: {
                    featureExtractor.extractEsfFeature(candidate.getSegmentCloud(), &feature);
                    Feature eigVal;
                    featureExtractor.extractEigenFeature(&candidate, &eigVal);
                    feature+=eigVal;
                    break;
                }
            }
            // Predicting the label using classifier
            string predictedClass;
            if(rForest){
                randomForest.predictClass(feature, predictedClass);
            } else{
                predictClassKnn(feature, predictedClass);
            }
            // Check if the cluster is pole. Else, it is detected as a tree
            if(predictedClass.find("Pole") != -1){
                detectedPoles->push_back(candidate);
            }else if (predictedClass.find("Tree") != -1){
                detectedTrees->push_back(candidate);
            }
        }
    }
    cerr << "Number of detected Pole-like Objects: " << poleLikeObjects->size() << endl;
    cerr << "Number of detected Poles: " << detectedPoles->size() << endl;
    cerr << "Number of detected Trees: " << detectedTrees->size() << endl;
}

void FeatureBasedClassifier::loadRandomForest(string pathToClassifier) {
    randomForest.loadClassifier(pathToClassifier);
}

void FeatureBasedClassifier::loadKnn(string pathToKnn, double knnThreshold_) {
    knnThreshold = knnThreshold_;
    /// Loading trained KdTree
    string kdtree_file = pathToKnn + "kdtree.idx";
    string training_data_h5_file = pathToKnn + "training_data.h5";
    string training_data_name_file = pathToKnn + "training_data.list";

    loadKnnLabels(training_data_name_file);
    // Reading Kd Tree from training file
    flann::Matrix<float> data;
    flann::load_from_file (data, training_data_h5_file, "training_data");
    kdTree = new flann::Index<flann::ChiSquareDistance<float> > (data, flann::SavedIndexParams (kdtree_file));
    kdTree->buildIndex();

}

void FeatureBasedClassifier::loadKnnLabels(string name_file){
    // Reading labels from training file
    ifstream fs;
    fs.open (name_file.c_str ());
    string line;
    while (!fs.eof ()){
        getline (fs, line);
        if (line.empty ())
            continue;
        knnLabelList.push_back(line);
    }
    fs.close ();
}

void FeatureBasedClassifier::predictClassKnn(Feature feature, string &predictedClass) {
    flann::Matrix<float> feature_flann = flann::Matrix<float>(new float[feature.signatureAsVector.size ()],
                                                              1, feature.signatureAsVector.size ());
    memcpy (&feature_flann.ptr ()[0], &feature.signatureAsVector[0],
            feature_flann.cols * feature_flann.rows * sizeof (float));

    // Calculating k nearest neighbours (k = 1)
    int k = 1;
    flann::Matrix<int> k_indices = flann::Matrix<int>(new int[k], 1, k);
    flann::Matrix<float> k_distances = flann::Matrix<float>(new float[k], 1, k);
    kdTree->knnSearch (feature_flann, k_indices, k_distances, k, flann::SearchParams (512));
    delete[] feature_flann.ptr ();

    // Checking if the mean distance is less than threshold
    double meanDist = k_distances[0][0];
    if( meanDist < knnThreshold){
        // Check if the cluster is pole. Else, it is detected as a tree
        predictedClass = knnLabelList.at(k_indices[0][0]);
    }
    else
        predictedClass = "rand";
}
