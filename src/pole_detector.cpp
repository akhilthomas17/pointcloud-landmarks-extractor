#include <pole_detector.h>


PCLPoleDetector::PCLPoleDetector(){
	inCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	processCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	debugCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
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
	pcl::PCDReader reader;
	reader.read (pathToFile, *inCloud);
	// pcl::io::loadPCDFile<pcl::PointXYZ>(pathToFile, *inCloud);
	// std::vector<int> indices;
	// pcl::removeNaNFromPointCloud(*inCloud, *inCloud, indices);
	// cerr << "PointCloud size before: " << inCloud->width * inCloud->height << " data points." << endl;
	// Should I make a copy? Is the nmodified input cloud required at any other stage?
	processCloud = inCloud;
	// pointCloudVisualizer(inCloud, 'r', "Input cloud");
}

void PCLPoleDetector::writePCD(string pathToFile){
	pcl::PCDWriter writer;
	writer.writeBinary(pathToFile, *debugCloud);
	// pcl::io::savePCDFileASCII (pathToFile, *processCloud);
  	std::cerr << "Saved " << debugCloud->points.size() << " data points to output_pcd.pcd." << std::endl;
}

void PCLPoleDetector::statistical_outlier_remover(double mean, double stdDev){
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (processCloud);
	sor.setMeanK (mean);
	sor.setStddevMulThresh (stdDev);
	sor.filter (*processCloud);
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

void PCLPoleDetector::preProcessor(double meanKNoise, double stdDevNoise, double distThreshold){
  	// Filtering the point cloud to remove outliers
  	// The mean and stdDev values have to be tuned for the particular case
  	statistical_outlier_remover(meanKNoise, stdDevNoise);
  	pcl::PointXYZ minPt , maxPt;
  	pcl::getMinMax3D(*processCloud, minPt, maxPt);

  	//pointCloudVisualizer(processCloud, 'g', "Noise removed cloud");
  	cerr << "PointCloud size after noise removal: " << processCloud->width * processCloud->height << " data points." << endl;
  	groundPlaneRemover(distThreshold);
  	cerr << "PointCloud size after ground plane removal: " << processCloud->width * processCloud->height << " data points." << endl;
}

void PCLPoleDetector::cutBuilder(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, double zMin, double zMax){
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (processCloud);
	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (zMin, zMax);
  	pass.filter(*cutCloud);
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
}

void makeColoredPoint(pcl::PointXYZRGB & PtColored, pcl::PointXYZ const & Pt, uint32_t rgb){
	PtColored.x = Pt.x;
	PtColored.y = Pt.y;
	PtColored.z = Pt.z;
	PtColored.rgb = *reinterpret_cast<float*>(&rgb);
}

void eigenV4f2PointXYZ(Eigen::Vector4f const &vec, pcl::PointXYZ &point){
	point.x = vec[0];
	point.y = vec[1];
	point.z = vec[2];
}

void PCLPoleDetector::clusterFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, vector<pcl::PointIndices> const &clusterIndices, double maxDist, list<Cluster> &filteredCluster){
  	int j = 0;
  	boost::random::uniform_int_distribution<> dist(0, 255);
  	for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
  	{
  		//** Creating the point cloud for each cluster
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    	//** Making color for each cluster
    	uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen); 
		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    	for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      		cloud_cluster->points.push_back (cutCloud->points[*pit]); //*
      		pcl::PointXYZRGB PtColored;
      		makeColoredPoint(PtColored, cutCloud->points[*pit], rgb);
      		debugCloud->points.push_back(PtColored);
    	}
    	cloud_cluster->width = cloud_cluster->points.size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;
    	//pointCloudVisualizer(cloud_cluster, 'b', "cluster" + boost::lexical_cast<string>(j));

    	//** Check if the cluster satisfies max-distance criterion
    	pcl::PointXYZ minPt, maxPt;
    	double maxDistCluster = pcl::getMaxSegment(*cloud_cluster, minPt, maxPt);
    	//cerr << "Max Distance of cluster " << j << ": " << maxDistCluster << endl;
    	if (maxDistCluster <= maxDist){
    		Eigen::Vector4f vecCentroid, maxDistVec;
    		pcl::compute3DCentroid(*cloud_cluster, vecCentroid);
    		pcl::getMaxDistance(*cloud_cluster, vecCentroid, maxDistVec);
    		double radius = (vecCentroid - maxDistVec).norm();
    		//cerr << "Radius of cluster " << j << ": " << radius << endl;
    		//Cluster cluster(vecCentroid, radius);
    		filteredCluster.push_back(Cluster(vecCentroid, radius));
    		j++;
    	}
   	}
}

void PCLPoleDetector::segmenterLanda(double numCuts, double minPts, double maxPts, double clusterTolerance){
	list<Cluster> filteredCluster;
	pcl::PointXYZ minPt , maxPt;
  	pcl::getMinMax3D(*processCloud, minPt, maxPt);
  	double maxHeight = maxPt.z;
  	double minHeight = minPt.z;
	double stepCut = (maxHeight - minHeight)/numCuts;
	int j = 0;
	int lastIterSize = 0;
	double maxDiameter = 10; // For post-filtering. To remove very large unprobable clusters
	for (int minCut = minHeight; minCut < maxHeight; minCut+=stepCut){
		pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud(new pcl::PointCloud<pcl::PointXYZ>);
		cutBuilder(cutCloud, minCut, minCut+stepCut);
		//** To visualize the cut (Random colour for each cut)
		pointCloudVisualizer(cutCloud, "Cut Cloud" + boost::lexical_cast<string>(j));

		cerr << "Size of cut " << j << ": " << cutCloud->width * cutCloud->height << " data points." << endl;
		vector<pcl::PointIndices> clusterIndices;
		euclideanClusterExtractor(cutCloud, clusterIndices, minPts, maxPts, clusterTolerance);
		cerr << "Number of clusters from cut " << j << ": " << clusterIndices.size() << endl;
		clusterFilter(cutCloud, clusterIndices, maxDiameter, filteredCluster);
		cerr << "Number of clusters from cut " << j << " after filtering: " << filteredCluster.size() - lastIterSize << endl;
		lastIterSize = filteredCluster.size();
		j++;
	}
	//pointCloudVisualizer(filteredCluster, 'r', "cluster");
}

void PCLPoleDetector::pointCloudVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string name){
	boost::random::uniform_int_distribution<> dist(0, 255);
	uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colourHandler(cloud, r, g, b);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, colourHandler, name);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

void PCLPoleDetector::pointCloudVisualizer(list<Cluster> const &clusterList, char colour, string id){
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
	int j = 0;
	pcl::PointXYZ center;
	for (list<Cluster>::const_iterator it = clusterList.begin(); it != clusterList.end(); ++it){
		Cluster cluster = *it;
		eigenV4f2PointXYZ(cluster.getCentroid(), center);
		viewer->addSphere(center, cluster.getRadius()/2, r, g, b, id+boost::lexical_cast<string>(j));
		j++;
	}

}

void PCLPoleDetector::algorithmLanda(string pathToPCDFile, double numCuts, double distThresholdCluster){
	readPCD(pathToPCDFile);
	double meanKNoise = 10;
	double stdDevNoise = 1;
	double distThresholdGround = 0.02;
	// preProcessor(meanKNoise, stdDevNoise, distThresholdGround);

	double minPts = 10;
	double maxPts = 500;
	segmenterLanda(numCuts, minPts, maxPts, distThresholdCluster);

	writePCD("output_pcd.pcd");
	while (!viewer->wasStopped ()) { // Display the visualiser until 'q' key is pressed
    	viewer->spinOnce (100);
    	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}