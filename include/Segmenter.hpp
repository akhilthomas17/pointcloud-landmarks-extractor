//
// Created by akhil on 18.05.17.
//

#ifndef POLE_DETECTOR_SEGMENTER_HPP
#define POLE_DETECTOR_SEGMENTER_HPP

#include <common.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <Visualizer.hpp>


class Segmenter {
public:
    Segmenter(pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud_){
        processCloud = processCloud_;
    }
    ~Segmenter(){}
    void euclideanSegmenter(double minPts, double maxPts, double clusterTolerance, double maxDiameter);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
    void euclideanClusterExtractor(vector<pcl::PointIndices> &clusterIndices,
                                   double minClusterSize, double maxClusterSize, double clusterTolerance);
    void clusterPreFilter(vector<pcl::PointIndices> const &clusterIndices, double maxDiameter);


};


#endif //POLE_DETECTOR_SEGMENTER_HPP
