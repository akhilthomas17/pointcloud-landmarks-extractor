//
// Created by akhil on 17.05.17.
//
#include "Preprocessor.hpp"

void Preprocessor::outlierRemover(int mean, double stdDev){
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (processCloud);
    sor.setMeanK (mean);
    sor.setStddevMulThresh (stdDev);
    sor.filter (*processCloud);
}

