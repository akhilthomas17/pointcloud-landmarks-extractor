//
// Created by akhil on 09.06.17.
//

#ifndef RULEBASEDDETECTION_HPP
#define RULEBASEDDETECTION_HPP

#include <Libs/Structures.hpp>

class RuleBasedDetection {
public:
    RuleBasedDetection(list<Segment>* detectedPoles_, list<Segment>* detectedTrees_):
    detectedPoles(detectedPoles_),
    detectedTrees(detectedTrees_){}
    void detectPoles(double minPoleHeight, double xyBoundThreshold, list<Segment> & stitchedClusters);
    void detectTrees(double minTreeHeight, double xyBoundMin, double xyBoundMax,
                     list<Segment> & stitchedClusters);
    void mergeExtractTrees(double maxDistance);

private:
    list<Segment>* detectedPoles;
    list<Segment>* detectedTrees;
};


#endif //POLE_DETECTOR_RULEBASEDDETECTION_HPP
