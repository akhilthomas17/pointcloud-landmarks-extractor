#include <pole_detector.h>


PCLPoleDetector::PCLPoleDetector(){
	inCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	processCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("Pole Detector"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0, "cloud", 0);
}

PCLPoleDetector::~PCLPoleDetector(){
	delete inCloud.get();
	delete processCloud.get();
	delete viewer.get();
}

void PCLPoleDetector::readPCD(string pathToFile){
	//pcl::PCDReader reader;
	//reader.read (pathToFile, *inCloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>(pathToFile, *inCloud);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*inCloud, *inCloud, indices);
	cerr << "PointCloud size before: " << inCloud->width * inCloud->height << " data points." << endl;
	processCloud = inCloud;
	pointCloudVisualizer(inCloud, 'r', "Input cloud");
}

void PCLPoleDetector::writePCD(string pathToFile){
	pcl::io::savePCDFileASCII (pathToFile, *processCloud);
  	std::cerr << "Saved " << processCloud->points.size() << " data points to output_pcd.pcd." << std::endl;
}

void PCLPoleDetector::heightThresholder(double minHeight, double maxHeight){
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (processCloud);
	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (minHeight, maxHeight);
  	pass.filter(*processCloud);
}

void PCLPoleDetector::statistical_outlier_remover(double mean, double stdDev){
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (processCloud);
	sor.setMeanK (mean);
	sor.setStddevMulThresh (stdDev);
	sor.filter (*processCloud);
}

void PCLPoleDetector::preProcessor(double groundClearance, double heightThreshold, double meanNoise, double stdDevNoise){
	pcl::PointXYZ minPt , maxPt;
	pcl::getMinMax3D(*processCloud, minPt, maxPt);
	cout << "---Before Noise Removal---" << std::endl;
	cout << "Min z: " << minPt.z << std::endl;
  	cout << "Max z: " << maxPt.z << std::endl;

  	// Filtering the point cloud to remove outliers
  	// The mean and stdDev values have to be tuned for the particular case
  	statistical_outlier_remover(meanNoise, stdDevNoise);
  	pcl::getMinMax3D(*processCloud, minPt, maxPt);
  	cout << "---After Noise Removal---" << std::endl;
	cout << "Min z: " << minPt.z << std::endl;
  	cout << "Max z: " << maxPt.z << std::endl;
  	pointCloudVisualizer(processCloud, 'g', "Noise removed cloud");

  	// Ground Removal and Height Thresholding: Points in the selected range of "z" (height) is preserved
  	// Aim: To remove ground points and to remove structures with height more than a normal pole
  	double minHeight = minPt.z + groundClearance;
  	double maxHeight = minPt.z + heightThreshold;
  	heightThresholder(minHeight, maxHeight);
  	cerr << "PointCloud size after filtering: " << processCloud->width * processCloud->height << " data points." << endl;
  	pointCloudVisualizer(processCloud, 'b', "Height thesholded cloud");
}

void PCLPoleDetector::pointCloudVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char colour, string name){
	int r, g, b;
	switch(colour){
		case 'r':
				r = 255;
				g = 0;
				b = 0;
				break;
		case 'g':
				r = 0;
				g = 255;
				b = 0;
				break;
		case 'b':
				r = 0;
				g = 0;
				b = 255;
				break;
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colourHandler(cloud, r, g, b);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, colourHandler, name);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

void PCLPoleDetector::engineLanda(string pathToPCDFile){
	readPCD(pathToPCDFile);
	preProcessor(0.1, 0.6, 50, 1);
	while (!viewer->wasStopped ()) { // Display the visualiser until 'q' key is pressed
    	viewer->spinOnce (100);
	}
}