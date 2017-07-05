//
// Created by akhil on 09.06.17.
//

#ifndef IO_HPP
#define IO_HPP

#include <pcl/io/pcd_io.h>
#include <Libs/Structures.hpp>

class IO{
public:
    IO(){}
    void readPCD(string pathToFile, pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud){
        reader.read (pathToFile, *inCloud);
        cerr << "Size of input cloud: " << inCloud->width * inCloud->height << " data points." << endl;
    }
    void readDon(string pathToFile, pcl::PointCloud<pcl::PointNormal>::Ptr donCloud){
        pcl::PCDReader reader;
        reader.read(pathToFile, *donCloud);
        cerr << "Size of DON input cloud: " << donCloud->width * donCloud->height << " data points." << endl;
    }
    void writePCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud, string pathToFile){
        writer.writeBinary(pathToFile, *outCloud);
        std::cerr << "Saved " << outCloud->points.size() << " data points to " + pathToFile << std::endl;
    }
    void writeSegments(list<Segment>& segmentList, string folderName){
        if(!segmentList.empty()){
            if(!boost::filesystem::exists(boost::filesystem::path(folderName)))
                boost::filesystem::create_directory(boost::filesystem::path(folderName));
            int i = 0;
            for (std::list<Segment>::iterator it = segmentList.begin(); it != segmentList.end(); ++it){
                writer.writeBinary(folderName + "/" + folderName + boost::lexical_cast<std::string>(i) + ".pcd", *(it->getSegmentCloud()));
                i += 1;
            }
        }
    }
    void writeDebugCloud(list<Segment>& segmentList, string pathToFile){
        if(!segmentList.empty()){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr debugCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            visualizer.makeColouredCloud(segmentList, debugCloud);
            writePCD(debugCloud, pathToFile);
        }
    }

private:
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    Visualizer visualizer;
};

#endif //POLE_DETECTOR_IO_HPP


