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



class Master
{
public:
    Master();
    ~Master();
    void runPoleDetector(string pathToPCDFile, double xyBoundThreshold,
                         double maxDistanceStitches, double minPoleHeight, double scaleSmall);
    void buildRefClusters(string pathToPCDFile, double maxDistanceStitches, double scaleSmall);
	void runLandmarkClassifier(string pathToPCDFile, string pathToClassifier, double donScaleSmall,
                               int mode, bool rForest, double knnThreshold);

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
