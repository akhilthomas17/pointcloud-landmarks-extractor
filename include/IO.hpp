//
// Created by akhil on 09.06.17.
//

#ifndef POLE_DETECTOR_IO_HPP
#define POLE_DETECTOR_IO_HPP

#include <pcl/io/pcd_io.h>
#include <Structures.hpp>

class IO{
public:
    void readPCD(string pathToFile, pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud){
        pcl::PCDReader reader;
        reader.read (pathToFile, *inCloud);
        cerr << "Size of input cloud: " << inCloud->width * inCloud->height << " data points." << endl;
    }
    void readDon(string pathToFile, pcl::PointCloud<pcl::PointNormal>::Ptr donCloud){
        pcl::PCDReader reader;
        reader.read(pathToFile, *donCloud);
        cerr << "Size of DON input cloud: " << donCloud->width * donCloud->height << " data points." << endl;
    }
};

#endif //POLE_DETECTOR_IO_HPP


