//
// Created by akhil on 17.05.17.
//

#ifndef PREPROCESSOR_HPP
#define PREPROCESSOR_HPP

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <Libs/Structures.hpp>



class Preprocessor {
public:
    Preprocessor(pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud_){
        processCloud = processCloud_;
    }
    ~Preprocessor(){}
    void outlierRemover(int mean, double stdDev);
    void groundPlaneRemover(double distThreshold);
    void cutCloudBuilder(pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud, double zMin, double zMax);
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
};


#endif //POLE_DETECTOR_PREPROCESSOR_HPP
