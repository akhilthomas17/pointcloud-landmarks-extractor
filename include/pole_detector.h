#ifndef POLE_DETECTOR_H
#define POLE_DETECTOR_H

// Include required classes
#include <Preprocessor.hpp>
#include <Segmenter.hpp>
#include <FeatureDescriptor.h>
#include <RandomForestLearner.hpp>
#include <Visualizer.hpp>
#include <IO.hpp>

// General include files:
#include <boost/lexical_cast.hpp>
#include <flann/io/hdf5.h>
#include <flann/flann.h>


// PCL specific include files:
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>
#include <pcl/console/print.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>


class PCLPoleDetector
{
public:
    PCLPoleDetector();
    ~PCLPoleDetector();
    void writePCD(string pathToFile);
    void writeSegments(list<Segment>& segmentList);
    void writePoles();
    void writeTrees();
    void treeExtractor(double maxDistanceTrees);
    void treeExtractor(double minTreeHeight, double xyBoundMin, double xyBoundMax);
    void clusterFilter(double minHeight, double xyBoundThreshold);
    void poleDetector(double minPoleHeight, double xyBoundThreshold);
    void algorithmSingleCut(string pathToPCDFile, double xyBoundThreshold,
                            double maxDistanceStitches, double minPoleHeight, double scaleSmall);
    void buildRefClusters(string pathToPCDFile, double maxDistanceStitches, double scaleSmall);
    void loadEsfData(vector<string> &labelList, string name_file);
 	bool isPole(flann::Matrix<int> const& k_indices, vector<string> const& labelList);
    void knnFinder(flann::Index<flann::ChiSquareDistance<float> > const &kdTree,
                   vector<string> const &labelList, double kdTreeThreshold, int mode);
    void algorithmFeatureDescriptorBased(string pathToPCDFile, string pathToDataFolder,
                                         double donScaleSmall, double kdTreeThreshold, int mode);
	void featureMatcher(RandomForestLearner& classifier, int mode);
	void algorithmClassifierBased(string pathToPCDFile, string pathToClassifier, double donScaleSmall, int mode);

private:

    IO ioManager;
	pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
	pcl::PointCloud<pcl::PointNormal>::Ptr _donCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitchedCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr poleLikeCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr poleCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr treeCloud;
	boost::random::mt19937 randomGen;
	list<Cluster> rawClusters;
	list<Segment> stitchedClusters;
	list<Segment> poleLikeClusters;
	list<Segment> detectedPoles;
	list<Segment> detectedTrees;
};

#endif
