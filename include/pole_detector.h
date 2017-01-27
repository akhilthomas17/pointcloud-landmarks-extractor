#ifndef POLE_DETECTOR_H
#define POLE_DETECTOR_H

// General include files:
#include <iostream>
#include <boost/lexical_cast.hpp>

using namespace std;

// PCL specific include files:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>



class Cluster
{
public:
	Cluster(Eigen::Vector4f centroid_, double radius_):
	centroid(centroid_), 
	radius(radius_){}
	~Cluster(){}
	Eigen::Vector4f getCentroid(){return centroid;}
	double getRadius(){return radius;}

private:
	Eigen::Vector4f centroid;
	double radius;
};

class PCLPoleDetector
{
public:
    PCLPoleDetector();
    ~PCLPoleDetector();
    void readPCD(string pathToFile);
    void writePCD(string pathToFile);
    void preProcessor(double groundClearance, double heightThreshold, double meanPtsNoise, double stdDevNoise);
    void segmenterLanda(double numCuts, double minPts, double maxPts, double maxDist);
    void algorithmLanda(string pathToPCDFile, double groundClearance, double heightThreshold, double minClusterSize, double maxDist);
    void pointCloudVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char colour, string id);
    void pointCloudVisualizer(list<Cluster> const &clusterList, char colour, string id);

private:
	void heightThresholder();
    void heightThresholder(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, double zMin, double zMax);
    void statisticalOutlierRemover(double mean, double sigma);
    void planarSurfaceRemover(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud);
    void euclideanClusterExtractor(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud,  vector<pcl::PointIndices> &clusterIndices, double minClusterSize, double maxClusterSize, double clusterTolerance);
    void clusterFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, vector<pcl::PointIndices> const &clusterIndices, double maxDist, list<Cluster> &filteredCluster);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
	double minHeight, maxHeight;
	vector<Cluster> candidateClusters;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif