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
	//pcl::PCDReader reader;
	//reader.read (pathToFile, *PCLPoleDetector::inCloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>(pathToFile, *PCLPoleDetector::inCloud);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*PCLPoleDetector::inCloud, *PCLPoleDetector::inCloud, indices);
	cerr << "PointCloud size before: " << PCLPoleDetector::inCloud->width * PCLPoleDetector::inCloud->height << " data points." << endl;
	PCLPoleDetector::processCloud = PCLPoleDetector::inCloud;
	//cerr << "z value: " << PCLPoleDetector::inCloud->points[100].z << endl;
}

void PCLPoleDetector::writePCD(string pathToFile){
	pcl::io::savePCDFileASCII (pathToFile, *PCLPoleDetector::processCloud);
  	std::cerr << "Saved " << PCLPoleDetector::processCloud->points.size() << " data points to output_pcd.pcd." << std::endl;
}

void PCLPoleDetector::removeGroundPoints_height(double minHeight){
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (PCLPoleDetector::inCloud);
	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (minHeight, 0);
  	pass.filter(*PCLPoleDetector::processCloud);
  	cerr << "PointCloud size after: " << PCLPoleDetector::processCloud->width * PCLPoleDetector::processCloud->height << " data points." << endl;
}

void PCLPoleDetector::preProcessor(double groundClearance, double heightThreshold){
	pcl::PointXYZ minPt , maxPt;
	pcl::getMinMax3D(*PCLPoleDetector::processCloud	, minPt, maxPt);
  	cout << "Max z: " << maxPt.z << std::endl;
  	cout << "Min z: " << minPt.z << std::endl;

  	// Ground Removal and Height Thresholding: Points in the selected range of "z" (height) is preserved
  	// Aim: To remove ground points and to remove structures with height more than a normal pole
  	double minHeight = minPt.z + groundClearance;
  	double maxHeight = minPt.z + heightThreshold;
  	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (PCLPoleDetector::inCloud);
	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (minHeight, maxHeight);
  	pass.filter(*PCLPoleDetector::processCloud);
  	cerr << "PointCloud size after: " << PCLPoleDetector::processCloud->width * PCLPoleDetector::processCloud->height << " data points." << endl;


}