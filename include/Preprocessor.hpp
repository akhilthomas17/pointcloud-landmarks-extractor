//
// Created by akhil on 17.05.17.
//

#ifndef POLE_DETECTOR_PREPROCESSOR_HPP
#define POLE_DETECTOR_PREPROCESSOR_HPP

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <Structures.hpp>



class Preprocessor {
public:
    Preprocessor(pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud_){
        processCloud = processCloud_;
    }
    ~Preprocessor(){}
    void outlierRemover(int mean, double stdDev);
    void groundPlaneRemover(int distThreshold);
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
};


#endif //POLE_DETECTOR_PREPROCESSOR_HPP
