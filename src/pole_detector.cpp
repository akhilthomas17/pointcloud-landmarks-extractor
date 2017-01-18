#include <pole_detector.h>


PCLPoleDetector::PCLPoleDetector(){
	PCLPoleDetector::inCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	PCLPoleDetector::processCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
}

PCLPoleDetector::~PCLPoleDetector(){
	delete PCLPoleDetector::inCloud.get();
	delete PCLPoleDetector::processCloud.get();
}

void PCLPoleDetector::readPCD(string pathToFile){
	pcl::PCDReader reader;
	reader.read (pathToFile, *PCLPoleDetector::inCloud);
	cerr << "PointCloud size before: " << PCLPoleDetector::inCloud->width * PCLPoleDetector::inCloud->height << " data points." << endl;
	//cerr << "z value: " << PCLPoleDetector::inCloud->points[100].z << endl;
}

void PCLPoleDetector::writePCD(string pathToFile){
	pcl::io::savePCDFileASCII (pathToFile, *PCLPoleDetector::processCloud);
  	std::cerr << "Saved " << PCLPoleDetector::processCloud->points.size() << " data points to output_pcd.pcd." << std::endl;
}

void PCLPoleDetector::removeGroundPoints_height(double minHeight){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (PCLPoleDetector::inCloud);
	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (minHeight, 0);
  	pass.filter(*PCLPoleDetector::processCloud);
  	cerr << "PointCloud size after: " << PCLPoleDetector::processCloud->width * PCLPoleDetector::processCloud->height << " data points." << endl;
}