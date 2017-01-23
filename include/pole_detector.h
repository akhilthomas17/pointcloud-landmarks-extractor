#ifndef POLE_DETECTOR_H
#define POLE_DETECTOR_H

// General include files:
#include <iostream>

using namespace std;

// PCL specific include files:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

class Cluster
{
public:
	Cluster(){
	}
	~Cluster(){
	}
	pcl::PointXYZ getCenter(){return center;}
	double getRadius(){return radius;}

private:
	pcl::PointXYZ center;
	double radius;
};

class PCLPoleDetector
{
public:
    PCLPoleDetector();
    ~PCLPoleDetector();
    void readPCD(string pathToFile);
    void writePCD(string pathToFile);
    void heightThresholder();
    void heightThresholder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double zMin, double zMax);
    void statistical_outlier_remover(double mean, double sigma);
    void preProcessor(double groundClearance, double heightThreshold, double meanNoise, double stdDevNoise);
    void segmenter_landa(double numCuts, double minPts, double maxPts, double maxDist);
    void pointCloudVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char colour, string name);
    void algorithmLanda(string pathToPCDFile, double groundClearance, double heightThreshold);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
	double minHeight, maxHeight;
	vector<Cluster> candidateClusters;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif