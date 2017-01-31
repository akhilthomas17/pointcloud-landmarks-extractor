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
	// cerr << "PointCloud size before: " << inCloud->width * inCloud->height << " data points." << endl;
	// Should I make a copy? Is the nmodified input cloud required at any other stage?
	processCloud = inCloud;
	// pointCloudVisualizer(inCloud, 'r', "Input cloud");
}

void PCLPoleDetector::writePCD(string pathToFile){
	pcl::io::savePCDFileASCII (pathToFile, *processCloud);
  	std::cerr << "Saved " << processCloud->points.size() << " data points to output_pcd.pcd." << std::endl;
}

void PCLPoleDetector::groundPlaneRemover(double distThreshold){
	// Create the segmentation object for the planar model and set all the parameters
  	pcl::SACSegmentation<pcl::PointXYZ> seg;
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (100);
  	seg.setDistanceThreshold (distThreshold);

  	int j=0, nr_points = (int) processCloud->points.size ();
  	while (processCloud->points.size () > 0.3 * nr_points)
  	{
    	// Segment the largest planar component from the remaining cloud
    	seg.setInputCloud (processCloud);
    	seg.segment (*inliers, *coefficients);
   		if (inliers->indices.size () == 0)
    	{
      		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      		break;
    	}

    	

    	// Extract the planar inliers from the input cloud
    	pcl::ExtractIndices<pcl::PointXYZ> extract;
    	extract.setInputCloud (processCloud);
    	extract.setIndices (inliers);
    	/*
    	extract.setNegative (false);

    	// Get the points associated with the planar surface
    	extract.filter (*cloud_plane);
    	pointCloudVisualizer(cloud_plane, 'r', "Ground plane removed" + boost::lexical_cast<string>(j));

		*/
    	// Remove the planar inliers, extract the rest
    	extract.setNegative (true);
    	extract.filter (*processCloud);
    	j++;
  	}

}

void PCLPoleDetector::statistical_outlier_remover(double mean, double stdDev){
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (processCloud);
	sor.setMeanK (mean);
	sor.setStddevMulThresh (stdDev);
	sor.filter (*processCloud);
}

void PCLPoleDetector::preProcessor(double meanKNoise, double stdDevNoise, double distThreshold){

	/* Uncomment for enabling noise remover part
  	// Filtering the point cloud to remove outliers
  	// The mean and stdDev values have to be tuned for the particular case
  	statistical_outlier_remover(meanKNoise, stdDevNoise);
  	pcl::PointXYZ minPt , maxPt;
  	pcl::getMinMax3D(*processCloud, minPt, maxPt);
  	*/

  	//pointCloudVisualizer(processCloud, 'g', "Noise removed cloud");
  	cerr << "PointCloud size after noise removal: " << processCloud->width * processCloud->height << " data points." << endl;
  	groundPlaneRemover(distThreshold);
  	cerr << "PointCloud size after ground plane removal: " << processCloud->width * processCloud->height << " data points." << endl;

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

void PCLPoleDetector::algorithmLanda(string pathToPCDFile, double distThreshold){
	readPCD(pathToPCDFile);
	double meanKNoise = 10;
	double stdDevNoise = 1;
	preProcessor(meanKNoise, stdDevNoise, distThreshold);
	writePCD("output_pcd.pcd");
	while (!viewer->wasStopped ()) { // Display the visualiser until 'q' key is pressed
    	viewer->spinOnce (100);
    	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}