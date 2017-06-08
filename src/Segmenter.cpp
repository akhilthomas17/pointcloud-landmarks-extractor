//
// Created by akhil on 18.05.17.
//

#include "Segmenter.hpp"
void Segmenter::euclideanSegmenter(double minPts, double maxPts, double clusterTolerance, double maxDiameter) {
    vector<pcl::PointIndices> clusterIndices;

    cerr << "Size of preProcessed cloud: " << processCloud->width * processCloud->height << " data points." << endl;
    euclideanClusterExtractor(clusterIndices, minPts, maxPts, clusterTolerance);
    cerr << "Number of clusters found: " << clusterIndices.size() << endl;

    //For post-filtering. To remove very large unprobable clusters
    clusterPreFilter(clusterIndices, maxDiameter);
    ///cerr << "Number of clusters after filtering: " << rawClusters.size() << endl;

}

void Segmenter::euclideanClusterExtractor(vector<pcl::PointIndices> &clusterIndices, double minClusterSize,
                                          double maxClusterSize, double clusterTolerance) {
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
        cloud_cluster->width = cloud_cluster->points.size ();
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
            //Cluster cluster(vecCentroid, radius);
            ///rawClusters.push_back(Cluster(vecCentroid, clusterDiameter/2, minPt, maxPt, cloud_cluster));

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