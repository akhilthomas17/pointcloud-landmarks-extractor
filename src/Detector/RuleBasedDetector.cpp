//
// Created by akhil on 09.06.17.
//

#include "Detector/RuleBasedDetector.hpp"

void RuleBasedDetection::detectPoles(double minPoleHeight, double xyBoundThreshold,
                                     list<Segment> &stitchedClusters) {
    for (list<Segment>::iterator it = stitchedClusters.begin(); it != stitchedClusters.end(); ++it){
        Segment candidatePole = *it;
        if (candidatePole.getHeight() > minPoleHeight){
            Eigen::Vector4f minVecX, maxVecX, minVecY, maxVecY;
            pcl::PointXYZ minPtX, maxPtX, minPtY, maxPtY;
            minPtY = minPtX = candidatePole.getMinPt();
            maxPtY = maxPtX = candidatePole.getMaxPt();
            minPtX.y = (minPtX.y + maxPtX.y)/2;
            maxPtX.y = minPtX.y;
            PointXYZ2eigenV4f2D(minVecX, minPtX);
            PointXYZ2eigenV4f2D(maxVecX, maxPtX);
            minPtY.x = (minPtY.x + maxPtY.x)/2;
            maxPtY.x = minPtY.x;
            PointXYZ2eigenV4f2D(minVecY, minPtY);
            PointXYZ2eigenV4f2D(maxVecY, maxPtY);

            double xyBound = std::max((maxVecX - minVecX).norm(), (maxVecY - minVecY).norm());
            if (xyBound <= xyBoundThreshold){
                detectedPoles->push_back(candidatePole);
            }
        }
    }
    cerr << "Number of detected pole like structures: " << detectedPoles->size() << endl;
}

void RuleBasedDetection::detectTrees(double minTreeHeight, double xyBoundMin, double xyBoundMax,
                                     list<Segment> &stitchedClusters) {
    for (list<Segment>::iterator it = stitchedClusters.begin(); it != stitchedClusters.end(); ++it){
        Segment candidateTree = *it;
        if (candidateTree.getHeight() > minTreeHeight){
            Eigen::Vector4f minVecX, maxVecX, minVecY, maxVecY;
            pcl::PointXYZ minPtX, maxPtX, minPtY, maxPtY;
            minPtY = minPtX = candidateTree.getMinPt();
            maxPtY = maxPtX = candidateTree.getMaxPt();
            minPtX.y = (minPtX.y + maxPtX.y)/2;
            maxPtX.y = minPtX.y;
            PointXYZ2eigenV4f2D(minVecX, minPtX);
            PointXYZ2eigenV4f2D(maxVecX, maxPtX);
            minPtY.x = (minPtY.x + maxPtY.x)/2;
            maxPtY.x = minPtY.x;
            PointXYZ2eigenV4f2D(minVecY, minPtY);
            PointXYZ2eigenV4f2D(maxVecY, maxPtY);

            double xyBound = std::max((maxVecX - minVecX).norm(), (maxVecY - minVecY).norm());
            if (xyBound <= xyBoundMax && xyBound >= xyBoundMin){
                detectedTrees->push_back(candidateTree);
            }
        }
    }
    cerr << "Number of detected tree like structures: " << detectedTrees->size() << endl;

}

void RuleBasedDetection::mergeExtractTrees(double maxDistanceTrees) {
    list<Segment>::iterator it, it2;
    it = detectedPoles->begin();
    int j = 1;
    while(it != detectedPoles->end()){
        Segment candidateTree = *it;
        bool tree = false;
        it2 = detectedPoles->begin();
        advance(it2, j);
        while (it2!= detectedPoles->end()){
            double diffDistance = (candidateTree.getCentroid() - it2->getCentroid()).norm();
            if (diffDistance < maxDistanceTrees){
                tree = true;
                candidateTree.mergeSegment(*it2);
                it2 = detectedPoles->erase(it2);
            }
            else
                ++it2;
        }
        if (tree){
            detectedTrees->push_back(candidateTree);
            it = detectedPoles->erase(it);
        }
        else{
            ++it;
            ++j;
        }
    }
    cerr << "Number of detected Poles: " << detectedPoles->size() << endl;
    cerr << "Number of detected Trees: " << detectedTrees->size() << endl;
}

