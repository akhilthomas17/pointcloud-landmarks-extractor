#ifndef POLE_DETECTOR_H
#define POLE_DETECTOR_H

// General include files:
#include <iostream>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>


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
    void pointCloudVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string name);
    void pointCloudVisualizer(list<Cluster> const &clusterList, char colour, string id);
    void segmenterLanda(double numCuts, double minPts, double maxPts, double clusterTolerance);
    void preProcessor(double meanKNoise, double stdDevNoise, double distThreshold);
    void algorithmLanda(string pathToPCDFile, double numCuts, double distThresholdCluster);

private:
	void readPCD(string pathToFile);
    void writePCD(string pathToFile);
    void groundPlaneRemover(double distThreshold);
    void statistical_outlier_remover(double mean, double sigma);
	void cutBuilder(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, double zMin, double zMax);
	void euclideanClusterExtractor(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, vector<pcl::PointIndices> &clusterIndices, double minClusterSize, double maxClusterSize, double clusterTolerance);
	void clusterFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, vector<pcl::PointIndices> const &clusterIndices, double maxDist, list<Cluster> &filteredCluster);

	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr debugCloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::random::mt19937 randomGen;
};

#endif