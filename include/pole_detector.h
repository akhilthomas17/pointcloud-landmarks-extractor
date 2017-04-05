#ifndef POLE_DETECTOR_H
#define POLE_DETECTOR_H

// General include files:
#include <iostream>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/lexical_cast.hpp>
#include <flann/io/hdf5.h>
#include <flann/flann.h>


using namespace std;

// PCL specific include files:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/common/angles.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/esf.h>



class Cluster
{
public:
	Cluster(Eigen::Vector4f centroid_, double radius_, pcl::PointXYZ minPt_, pcl::PointXYZ maxPt_, pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud_);
	Cluster();
	~Cluster(){}
	Eigen::Vector4f const& getCentroid(){return centroid;}
	double const& getRadius(){return radius;}
	double getZMin(){return minPt.z;}
	double getZMax(){return maxPt.z;}
	pcl::PointXYZ const& getMinPt(){return minPt;}
	pcl::PointXYZ const& getMaxPt(){return maxPt;}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getClusterCloud(){return clusterCloud;}
	bool const& isProcessed(){return processed;}
	void markProcessed(){processed = true;}

private:
	Eigen::Vector4f centroid;
	double radius;
	pcl::PointXYZ minPt;
	pcl::PointXYZ maxPt;
	bool processed;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud;
	
};


class Segment
{
public:
	Segment();
	~Segment(){segmentParts.clear();};
	void mergeSegment(Segment& segment);
	void addCluster(Cluster& cluster);
	double getHeight(){return height;}
	double getZMax(){return maxPt.z;}
	double getZMin(){return minPt.z;}
	pcl::PointXYZ const& getMinPt(){return minPt;}
	pcl::PointXYZ const& getMaxPt(){return maxPt;}
	list<Cluster>& getSegmentParts(){return segmentParts;}
	Eigen::Vector4f getCentroid(){return centroid;}
	Eigen::Vector4f getBase(){return base;}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSegmentCloud(){return segmentCloud;}
	

private:
	list<Cluster> segmentParts;
	Eigen::Vector4f centroid;
	double height;
	pcl::PointXYZ minPt;
	pcl::PointXYZ maxPt;
	Eigen::Vector4f base;
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud;
	
};


class PCLPoleDetector
{
public:
    PCLPoleDetector();
    ~PCLPoleDetector();
    void readPCD(string pathToFile);
    void statisticalOutlierRemover(double mean, double sigma);
    void writePCD(string pathToFile);
    void writePoles();
    void writeTrees();
    void pointCloudVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string name);
    void pointCloudVisualizer(list<Cluster> const &clusterList, string id);
    void pointCloudVisualizer(Cluster &cluster, string id);
    void preProcessor(double meanKNoise, double stdDevNoise, double distThreshold);
    void segmenterSingleCut(double minPts, double maxPts, double clusterTolerance, double maxDiameter);
    void segmenterDON(double minPts, double maxPts, double scaleSmall, double scaleLarge, double thresholdDON);
    void treeExtractor(double maxDistanceTrees);
    void treeExtractor(double minTreeHeight, double xyBoundMin, double xyBoundMax);
    void clusterStitcher(double angleToVertical, double maxDistanceStitches);
    void clusterFilter(double minHeight, double xyBoundThreshold);
    void poleDetector(double minPoleHeight, double xyBoundThreshold);
    void algorithmSingleCut(string pathToPCDFile, double xyBoundThreshold, double maxDistanceStitches, double minPoleHeight, double scaleSmall);
    void buildRefClusters(string pathToPCDFile, double maxDistanceStitches, double scaleSmall);
    void loadEsfData(vector<string> &labelList, string name_file);
 	bool isPole(flann::Matrix<int> const& k_indices, vector<string> const& labelList);
    void esfMatcher(flann::Index<flann::ChiSquareDistance<float> > const& kdTree, vector<string> const& labelList, double kdTreeThreshold);
    void algorithmEsfBased(string pathToPCDFile, string pathToDataFolder, double donScaleSmall, double kdTreeThreshold);

private:
	
	void readDON(string pathToFile);
    void groundPlaneRemover(double distThreshold);
	void euclideanClusterExtractor(vector<pcl::PointIndices> &clusterIndices, double minClusterSize, double maxClusterSize, double clusterTolerance);
	void euclideanClusterExtractor(pcl::PointCloud<pcl::PointNormal>::Ptr donCloud, vector<pcl::PointIndices> &clusterIndices, double minClusterSize, double maxClusterSize, double clusterTolerance);
	void clusterFilter(vector<pcl::PointIndices> const &clusterIndices, double maxDiameter);
	void clusterCloudBuilder(vector<pcl::PointIndices> const &clusterIndices);
	void DONBuilder(pcl::PointCloud<pcl::PointNormal>::Ptr donCloud, double scaleSmall, double scaleLarge);
	void DONThresholder(pcl::PointCloud<pcl::PointNormal>::Ptr donCloud, double thresholdDON);

	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
	pcl::PointCloud<pcl::PointNormal>::Ptr _donCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitchedCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr poleLikeCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr poleCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr treeCloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::random::mt19937 randomGen;
	list<Cluster> rawClusters;
	list<Segment> stitchedClusters;
	list<Segment> poleLikeClusters;
	list<Segment> detectedPoles;
	list<Segment> detectedTrees;
};

#endif