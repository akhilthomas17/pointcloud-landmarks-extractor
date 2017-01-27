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
	cerr << "PointCloud size original: " << inCloud->width * inCloud->height << " data points." << endl;
	processCloud = inCloud;
	//pointCloudVisualizer(inCloud, 'r', "Input cloud");
}

void PCLPoleDetector::writePCD(string pathToFile){
	pcl::io::savePCDFileASCII (pathToFile, *processCloud);
  	std::cerr << "Saved " << processCloud->points.size() << " data points to output_pcd.pcd." << std::endl;
}

void PCLPoleDetector::heightThresholder(){
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (processCloud);
	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (minHeight, maxHeight);
  	pass.filter(*processCloud);
  	cerr << "PointCloud size after height thresholding: " << processCloud->width * processCloud->height << " data points." << endl;
}

void PCLPoleDetector::heightThresholder(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, double zMin, double zMax){
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (processCloud);
	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (zMin, zMax);
  	pass.filter(*cutCloud);
}

void PCLPoleDetector::statisticalOutlierRemover(double mean, double stdDev){
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (processCloud);
	sor.setMeanK (mean);
	sor.setStddevMulThresh (stdDev);
	sor.filter (*processCloud);
	cerr << "PointCloud size after noise removal: " << processCloud->width * processCloud->height << " data points." << endl;
}

void PCLPoleDetector::preProcessor(double groundClearance, double heightThreshold, double meanPtsNoise, double stdDevNoise){
	pcl::PointXYZ minPt , maxPt;

	/*
	pcl::getMinMax3D(*processCloud, minPt, maxPt);
	cout << "---Before Noise Removal---" << std::endl;
	cout << "Min z: " << minPt.z << std::endl;
  	cout << "Max z: " << maxPt.z << std::endl;
  	*/

  	// Filtering the point cloud to remove outliers
  	// The meanPtsNoise and stdDev values have to be tuned for the particular case
  	statisticalOutlierRemover(meanPtsNoise, stdDevNoise);
  	pcl::getMinMax3D(*processCloud, minPt, maxPt);
  	/*
  	cout << "---After Noise Removal---" << std::endl;
	cout << "Min z: " << minPt.z << std::endl;
  	cout << "Max z: " << maxPt.z << std::endl;
  	*/
  	//pointCloudVisualizer(processCloud, 'g', "Noise removed cloud");

  	// Ground Removal and Height Thresholding: Points in the selected range of "z" (height) is preserved
  	// Aim: To remove ground points and to remove structures with height more than a normal pole
  	minHeight = minPt.z + groundClearance;
  	maxHeight = minPt.z + heightThreshold;
  	heightThresholder();
  	//cerr << "PointCloud size after Preprocessing: " << processCloud->width * processCloud->height << " data points." << endl;
  	//pointCloudVisualizer(processCloud, 'b', "Height thesholded cloud");
}

void PCLPoleDetector::planarSurfaceRemover(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud){
	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ()); //To see the extracted plane
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (10);
	seg.setDistanceThreshold (0.02);

	int i=0, nr_points = (int) cutCloud->points.size ();
  	while (cutCloud->points.size () > 0.3 * nr_points){
    	// Segment the largest planar component from the remaining cloud
    	seg.setInputCloud (cutCloud);
    	seg.segment (*inliers, *coefficients);
    	if (inliers->indices.size () == 0){
      		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      		break;
    	}

    	// Extract the planar inliers from the input cloud
    	pcl::ExtractIndices<pcl::PointXYZ> extract;
    	extract.setInputCloud (cutCloud);
    	extract.setIndices (inliers);
    	extract.setNegative (false);

    	/* To see the plane extracted, uncomment below:

    	// Get the points associated with the planar surface
    	extract.filter (*cloud_plane);
    	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    	*/

    	// Remove the planar inliers, extract the rest
    	extract.setNegative (true);
    	extract.filter (*cutCloud);
  	}
}

void PCLPoleDetector::euclideanClusterExtractor(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, vector<pcl::PointIndices> &clusterIndices, double minClusterSize, double maxClusterSize, double clusterTolerance){
	// Creating the KdTree object for the search method of the extraction
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cutCloud);

  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setClusterTolerance (clusterTolerance);
  	ec.setMinClusterSize (minClusterSize);
  	ec.setMaxClusterSize (maxClusterSize);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cutCloud);
  	ec.extract (clusterIndices);
  	cerr << "Number of clusters for cut-0 " << clusterIndices.size() << endl;

   	int j = 0;
  	for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
  	{
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    	for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      		cloud_cluster->points.push_back (cutCloud->points[*pit]); //*
    	cloud_cluster->width = cloud_cluster->points.size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;
    	pointCloudVisualizer(cloud_cluster, 'b', "cluster" + boost::lexical_cast<string>(j));
    	cout << "PointCloud representing the Cluster " << j << ":" << cloud_cluster->points.size () << " data points." << std::endl;
    	j++;
  	}
}

void PCLPoleDetector::clusterFilter(vector<pcl::PointIndices> const &clusterIndices, double maxDist, list<Cluster> finalCluster){

}

void PCLPoleDetector::segmenterLanda(double numCuts, double minPts, double maxPts, double maxDist){
	double stepCut = (maxHeight - minHeight)/numCuts;
	double clusterTolerance = 1;
	for (int minCut = minHeight; minCut < maxHeight; minCut+=stepCut){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud(new pcl::PointCloud<pcl::PointXYZ>);
		heightThresholder(cutCloud, minCut, minCut+stepCut);
		// pointCloudVisualizer(cutCloud, 'g', "Cut Cloud");
		planarSurfaceRemover(cutCloud);
		pointCloudVisualizer(cutCloud, 'g', "Plane removed Cut Cloud");
		cerr << "PointCloud size of cutCloud 0 " << cutCloud->width * cutCloud->height << " data points." << endl;
		vector<pcl::PointIndices> clusterIndices;
		euclideanClusterExtractor(cutCloud, clusterIndices, minPts, maxPts, clusterTolerance);
		break;
	}
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

void PCLPoleDetector::algorithmLanda(string pathToPCDFile, double groundClearance, double heightThreshold, double minClusterSize, double maxDist){
	readPCD(pathToPCDFile);
	double minKNoise = 10;
	double stdDevNoise = 1;
	preProcessor(groundClearance, heightThreshold, minKNoise, stdDevNoise);
	segmenterLanda(heightThreshold*0.8, 10, 1000, maxDist);
	while (!viewer->wasStopped ()) { // Display the visualiser until 'q' key is pressed
    	viewer->spinOnce (100);
    	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}