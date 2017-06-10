//
// Created by akhil on 17.05.17.
//

#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <Libs/Structures.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

class Visualizer {
public:

    Visualizer(){
    }

    void initializeViewer(){
        viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("Pole Detector"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (3.0, "coordinates", 0);
    }

    void viewCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string name){
        boost::random::uniform_int_distribution<> dist(0, 255);
        uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colourHandler(cloud, r, g, b);
        viewer->addPointCloud<pcl::PointXYZ> (cloud, colourHandler, name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);

    }


    void viewClusterList(list<Cluster> const &clusterList, string id){
        boost::random::uniform_int_distribution<> dist(0, 255);
        uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
        int j = 0;
        pcl::PointXYZ center;
        for (list<Cluster>::const_iterator it = clusterList.begin(); it != clusterList.end(); ++it){
            Cluster cluster = *it;
            eigenV4f2PointXYZ(cluster.getCentroid(), center);
            viewer->addSphere(center, cluster.getRadius()/2, r, g, b, id+boost::lexical_cast<string>(j));
            j++;
        }
    }

    void viewCluster(Cluster &cluster, string id){
        boost::random::uniform_int_distribution<> dist(0, 255);
        uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
        pcl::PointXYZ center;
        eigenV4f2PointXYZ(cluster.getCentroid(), center);
        viewer->addSphere(center, cluster.getRadius()/2, r, g, b, id);
    }

    void makeColouredCloud(list<Segment>& segmentList, pcl::PointCloud<pcl::PointXYZRGB>::Ptr debugCloud){
        boost::random::uniform_int_distribution<> dist(0, 255);
        for (list<Segment>::iterator it = segmentList.begin(); it != segmentList.end(); ++it) {
            Segment segment = *it;
            //** Making single color for each detected pole
            uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
            uint32_t rgb = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
            for (list<Cluster>::iterator it2 = segment.getSegmentParts().begin();
                 it2 != segment.getSegmentParts().end(); ++it2) {
                for (size_t i = 0; i < it2->getClusterCloud()->points.size(); ++i) {
                    pcl::PointXYZRGB PtColored;
                    makeColoredPoint(PtColored, it2->getClusterCloud()->points[i], rgb);
                    debugCloud->points.push_back(PtColored);
                }
            }
        }
    }

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::random::mt19937 randomGen;

};


#endif //POLE_DETECTOR_VISUALIZER_HPP
