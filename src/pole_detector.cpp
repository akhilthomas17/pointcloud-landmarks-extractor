#include <pole_detector.h>


PCLPoleDetector::PCLPoleDetector(){
	inCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	processCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
}

PCLPoleDetector::~PCLPoleDetector(){
	delete inCloud.get();
	delete processCloud.get();
}

void PCLPoleDetector::readPCD(string pathToFile){
	//pcl::PCDReader reader;
	//reader.read (pathToFile, *inCloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>(pathToFile, *inCloud);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*inCloud, *inCloud, indices);
	cerr << "PointCloud size before: " << inCloud->width * inCloud->height << " data points." << endl;
	processCloud = inCloud;
}

void PCLPoleDetector::writePCD(string pathToFile){
	pcl::io::savePCDFileASCII (pathToFile, *processCloud);
  	std::cerr << "Saved " << processCloud->points.size() << " data points to output_pcd.pcd." << std::endl;
}

void PCLPoleDetector::removeGroundPoints_height(double minHeight){
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (inCloud);
	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (minHeight, 0);
  	pass.filter(*processCloud);
  	cerr << "PointCloud size after: " << processCloud->width * processCloud->height << " data points." << endl;
}

void PCLPoleDetector::preProcessor(double groundClearance, double heightThreshold){
	pcl::PointXYZ minPt , maxPt;
	pcl::getMinMax3D(*processCloud	, minPt, maxPt);
  	cout << "Max z: " << maxPt.z << std::endl;
  	cout << "Min z: " << minPt.z << std::endl;

  	// Ground Removal and Height Thresholding: Points in the selected range of "z" (height) is preserved
  	// Aim: To remove ground points and to remove structures with height more than a normal pole
  	double minHeight = minPt.z + groundClearance;
  	double maxHeight = minPt.z + heightThreshold;
  	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (inCloud);
	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (minHeight, maxHeight);
  	pass.filter(*processCloud);
  	cerr << "PointCloud size after: " << processCloud->width * processCloud->height << " data points." << endl;
}