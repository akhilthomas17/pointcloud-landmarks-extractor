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


class PCLPoleDetector
{
public:
    PCLPoleDetector();
    ~PCLPoleDetector();
    void readPCD(string pathToFile);
    void writePCD(string pathToFile);
    void removeGroundPoints_height(double minHeight);
    void preProcessor(double groundClearance, double heightThreshold);
    void pointCloudVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char colour, string name);
    void engineLanda(string pathToPCDFile);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

#endif