//
// Created by akhil on 18.05.17.
//

#include "Segmenter/Segmenter.hpp"

void Segmenter::stitchClusters(double angleToVertical, double maxDistanceStitches,
                               list<Segment> &stitchedClusters) {
    boost::random::uniform_int_distribution<> dist(0, 255);
    rawClusters->sort(compareClusterHeight);

    /* To check the input clusters
    int j = 0;
    for (list<Cluster>::iterator it = rawClusters.begin(); it != rawClusters.end(); ++it){
        Cluster cluster = *it;
        cerr << "Radius: " << cluster.getRadius() << endl;
        cerr << "Z min: " << cluster.getZMin() << endl;
        cerr << "Z max: " << cluster.getZMax() << endl;
        pointCloudVisualizer(cluster.getClusterCloud(), "cluster cloud " + boost::lexical_cast<string>(j));
        pointCloudVisualizer(cluster, "cluster shape " + boost::lexical_cast<string>(j));
        j++;
        break;
    }
    */

    int numClusters = 0;
    for (list<Cluster>::iterator it = rawClusters->begin(); it != rawClusters->end(); ++it){
        if (! it->isProcessed()){
            it->markProcessed();
            Segment stitchCluster;
            stitchCluster.addCluster(*it);
            //pointCloudVisualizer(it->getClusterCloud(), "cluster cloud " + boost::lexical_cast<string>(j));
            //pointCloudVisualizer(*it, "cluster shape " + boost::lexical_cast<string>(j));
            for (list<Cluster>::iterator it2 = rawClusters->begin(); it2 != rawClusters->end(); ++it2){
                if (! it2->isProcessed()){
                    //pointCloudVisualizer(it2->getClusterCloud(), "cluster cloud " + boost::lexical_cast<string>(j));
                    //pointCloudVisualizer(*it2, "cluster shape " + boost::lexical_cast<string>(j));
                    double heightDiff = it2->getZMin() - stitchCluster.getZMax();
                    //double heightDiff = it2->getCentroid()[2] - stitchCluster.getCentroid()[2];
                    //double heightDiff = it2->getCentroid()[2] - stitchCluster.getZMax();
                    if (heightDiff < maxDistanceStitches && heightDiff > 0){
                        Eigen::Vector4f diffVec = it2->getCentroid() - stitchCluster.getCentroid();
                        //** Hardcoded value below for Euclidean distance threshold of clusters to be stitched
                        if (diffVec.norm() < 5){
                            //double angle = abs(pcl::normAngle(diffVec.dot(Eigen::Vector4f::UnitZ())/diffVec.norm()));
                            double angle = pcl::getAngle3D(diffVec, Eigen::Vector4f::UnitZ());
                            if (angle < angleToVertical){
                                /* Debugs
                                cerr << "heightDiff: " << heightDiff << endl;
                                cerr << "Euclidean distance: " << diffVec.norm() << endl;
                                cerr << "Angle: " << angle << endl;
                                */
                                it2->markProcessed();
                                stitchCluster.addCluster(*it2);
                            }
                        }
                    }
                }
            }
            stitchedClusters.push_back(stitchCluster);
        }
    }
    cerr << "Number of clusters after stitching: " << stitchedClusters.size() << endl;

}

void Segmenter::donSegmenter(int minPts, int maxPts, double scaleSmall,
                                   double scaleLarge, double donThreshold, bool donLoaded){
    // Check if DON has to be build or is loaded from file
    if (!donLoaded)
        buildDon(scaleSmall, scaleLarge);
    thresholdDon(0.25);
    pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*donCloud, *processCloud);

    vector<pcl::PointIndices> clusterIndices;
    donClusterExtractor(clusterIndices, minPts, maxPts, donThreshold);

    clusterCloudBuilder(clusterIndices);
    cerr << "Number of clusters found: " << rawClusters->size() << endl;
}


void Segmenter::loadDon(string pathToPcd){
    ioManager.readDon(pathToPcd, donCloud);
}

void Segmenter::buildDon(double scaleSmall, double scaleLarge) {
    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    if (processCloud->isOrganized ()){
        tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    }
    else{
        tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud (processCloud);

    if (scaleSmall >= scaleLarge){
        cerr << "Error: Large scale must be > small scale!" << endl;
        exit (EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud (processCloud);
    ne.setSearchMethod (tree);

    /**
     * NOTE: setting viewpoint is very important, so that we can ensure
     * normals are all pointed in the same direction!
     */
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

    // calculate normals with the small scale
    cout << "Calculating normals for scale..." << scaleSmall << endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (scaleSmall);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    cout << "Calculating normals for scale..." << scaleLarge << endl;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

    ne.setRadiusSearch (scaleLarge);
    ne.compute (*normals_large_scale);

    cout << "Calculating DoN... " << endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
    don.setInputCloud (processCloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
        std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
        exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*donCloud);

    //* To write DON cloud to file, uncomment below
    // Save DoN features
    pcl::PCDWriter writer;
    writer.write<pcl::PointNormal> ("don.pcd", *donCloud, true);
    //*/
}

void Segmenter::thresholdDon(double thresholdDON){
    // Implementing the thesholding of DON
    // Build the condition for filtering
    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
            new pcl::ConditionOr<pcl::PointNormal> ()
    );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
            new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, thresholdDON))
    );
    // Build the filter
    pcl::ConditionalRemoval<pcl::PointNormal> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud (donCloud);
    // Apply filter
    condrem.filter (*donCloud);

    std::cout << "Filtered Pointcloud: " << donCloud->points.size () << " data points." << std::endl;

    // Save filtered output
    /*
    pcl::PCDWriter writer;
    writer.write<pcl::PointNormal> ("don_filtered.pcd", *donCloud, true);
    //*/
}

void Segmenter::donClusterExtractor(vector<pcl::PointIndices> &clusterIndices, int minClusterSize,
                                    int maxClusterSize, double clusterTolerance) {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud (donCloud);

    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (donCloud);
    ec.extract (clusterIndices);
}

void Segmenter::clusterCloudBuilder(vector<pcl::PointIndices> const &clusterIndices) {
    boost::random::uniform_int_distribution<> dist(0, 255);
    for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it){
        //** Creating the point cloud for each cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_cluster->points.push_back (processCloud->points[*pit]); //*
        }
        cloud_cluster->width = (uint32_t) cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ minPt, maxPt;
        Eigen::Vector4f minVec, maxVec;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        PointXYZ2eigenV4f2D(minVec, minPt);
        PointXYZ2eigenV4f2D(maxVec, maxPt);
        double xyDiameter = (maxVec - minVec).norm();

        Eigen::Vector4f vecCentroid;
        pcl::compute3DCentroid(*cloud_cluster, vecCentroid);
        rawClusters->push_back(Cluster(vecCentroid, xyDiameter/2, minPt, maxPt, cloud_cluster));
    }

}

void Segmenter::euclideanSegmenter(int minPts, int maxPts, double clusterTolerance, double maxDiameter) {
    vector<pcl::PointIndices> clusterIndices;

    cerr << "Size of preProcessed cloud: " << processCloud->width * processCloud->height << " data points." << endl;
    euclideanClusterExtractor(clusterIndices, minPts, maxPts, clusterTolerance);
    cerr << "Number of clusters found: " << clusterIndices.size() << endl;

    //Pre-filtering to remove very large unprobable clusters
    clusterPreFilter(clusterIndices, maxDiameter);
    ///cerr << "Number of clusters after filtering: " << rawClusters.size() << endl;

}

void Segmenter::euclideanClusterExtractor(vector<pcl::PointIndices> &clusterIndices, int minClusterSize,
                                          int maxClusterSize, double clusterTolerance) {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (processCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (processCloud);
    ec.extract (clusterIndices);

}

void Segmenter::clusterPreFilter(vector<pcl::PointIndices> const &clusterIndices, double maxDiameter) {
    int j = 0;
    boost::random::uniform_int_distribution<> dist(0, 255);
    for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it){
        //** Creating the point cloud for each cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_cluster->points.push_back (processCloud->points[*pit]); //*
        }
        cloud_cluster->width = (uint32_t) cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        //pointCloudVisualizer(cloud_cluster, 'b', "cluster" + boost::lexical_cast<string>(j));

        //** Check if the cluster satisfies Bounding box max-Diameter criterion
        pcl::PointXYZ minPt, maxPt;
        Eigen::Vector4f minVec, maxVec;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        PointXYZ2eigenV4f2D(minVec, minPt);
        PointXYZ2eigenV4f2D(maxVec, maxPt);
        double clusterDiameter = (maxVec - minVec).norm();
        //cerr << "Max Distance of cluster " << j << ": " << maxDistCluster << endl;
        if (clusterDiameter <= maxDiameter){
            Eigen::Vector4f vecCentroid;
            pcl::compute3DCentroid(*cloud_cluster, vecCentroid);
            //cerr << "Radius of cluster " << j << ": " << radius << endl;
            rawClusters->push_back(Cluster(vecCentroid, clusterDiameter/2, minPt, maxPt, cloud_cluster));

            /* Uncomment to save the un-stitched clusters
            //** Making color for each cluster
            uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            //**
            for (size_t i = 0; i < cloud_cluster->points.size(); ++i){
                pcl::PointXYZRGB PtColored;
                  makeColoredPoint(PtColored, cloud_cluster->points[i], rgb);
                  clusterCloud->points.push_back(PtColored);
            }
            //*/
            j++;
        }
    }
}
