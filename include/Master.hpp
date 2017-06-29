#ifndef MASTER_HPP
#define MASTER_HPP

// Include required classes
#include <Preprocessor/Preprocessor.hpp>
#include <Segmenter/Segmenter.hpp>
#include <Classifier/FeatureExtractor.hpp>
#include <Classifier/RandomForestLearner.hpp>
#include <Libs/Visualizer.hpp>
#include <Libs/IO.hpp>
#include "Detector/RuleBasedDetector.hpp"
#include <Classifier/FeatureBasedClassifier.hpp>


// General include files:
#include <boost/lexical_cast.hpp>


// PCL specific include files:
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>
#include <pcl/console/print.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>


/**
 * Class that has different approaches implemented for landmark classification from pointclouds
 */
class Master
{
public:
    Master();
    ~Master();
    void runInitialApproach(string pathToPCDFile, double xyBoundThreshold,
                           double maxDistanceStitches, double minPoleHeight);
    void runPoleDetector(bool donInput, string pathToPCDFile, double donThreshold, double maxDistanceStitches,
                             double xyBoundThreshold, double minPoleHeight);
    void buildRefClusters(string pathToPCDFile, double maxDistanceStitches, double scaleSmall);
	void runLandmarkClassifier(bool rawPCDinput, string pathToPCDFile, double donScaleSmall, int featureMode,
                               bool knnClassifier, string pathToClassifier, double knnThreshold);

private:

    IO ioManager;
	pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud;
	boost::random::mt19937 randomGen;
	list<Cluster> rawClusters;
	list<Segment> stitchedClusters;
	list<Segment> poleLikeClusters;
	list<Segment> detectedPoles;
	list<Segment> detectedTrees;
};

#endif
