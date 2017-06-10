//
// Created by akhil on 18.05.17.
//

#ifndef SEGMENTER_HPP
#define SEGMENTER_HPP

#include <Libs/Structures.hpp>
#include <Libs/Visualizer.hpp>
#include <Libs/IO.hpp>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>



class Segmenter {
public:
    Segmenter(pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud_, list<Cluster>* rawClusters_){
        processCloud = processCloud_;
        rawClusters = rawClusters_;
        donCloud = pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal>);
    }
    ~Segmenter(){}
    void euclideanSegmenter(int minPts, int maxPts, double clusterTolerance, double maxDiameter);
    void donSegmenter(int minPts, int maxPts, double scaleSmall,
                      double scaleLarge, double donThreshold, bool donLoaded);
    void loadDon(string pathToPcd);
    void stitchClusters(double angleToVertical, double maxDistanceStitches, list<Segment>& stitchedClusters);

private:
    IO ioManager;
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr donCloud;
    list<Cluster>* rawClusters;
    boost::random::mt19937 randomGen;

    void buildDon(double scaleSmall, double scaleLarge);
    void thresholdDon(double thresholdDon);
    void donClusterExtractor(vector<pcl::PointIndices> &clusterIndices, int minClusterSize,
                             int maxClusterSize, double clusterTolerance);
    void clusterCloudBuilder(vector<pcl::PointIndices> const &clusterIndices);

    void euclideanClusterExtractor(vector<pcl::PointIndices> &clusterIndices,
                                   int minClusterSize, int maxClusterSize, double clusterTolerance);
    void clusterPreFilter(vector<pcl::PointIndices> const &clusterIndices, double maxDiameter);


};


#endif //POLE_DETECTOR_SEGMENTER_HPP
