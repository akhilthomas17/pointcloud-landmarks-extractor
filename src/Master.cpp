#include <Master.hpp>

Master::Master(){
	processCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
}


Master::~Master(){
	rawClusters.clear();
	stitchedClusters.clear();
	poleLikeClusters.clear();
	detectedPoles.clear();
	detectedTrees.clear();
}


void Master::runPoleDetector(string pathToPCDFile, double xyBoundThreshold,
                                      double maxDistanceStitches, double minPoleHeight, double scaleSmall){
	//ioManager.readPCD(pathToPCDFile, processCloud);
	//ioManager.readDon(pathToPCDFile, _donCloud);
	double meanKNoise = 10;
	double stdDevNoise = 1;
	double distThresholdGround = 0.02;

    /* Uncomment to do preprocessing
     // Filtering the point cloud to remove outliers
  	// The mean and stdDev values have to be tuned for the particular case
    Preprocessor preprocessor(processCloud);
    preprocessor.outlierRemover(meanKNoise, stdDevNoise);
  	pcl::PointXYZ minPt , maxPt;
  	pcl::getMinMax3D(*processCloud, minPt, maxPt);

  	//pointCloudVisualizer(processCloud, 'g', "Noise removed cloud");
  	cerr << "PointCloud size after noise removal: " << processCloud->width * processCloud->height << " data points." << endl;
  	preprocessor.groundPlaneRemover(distThreshold);
  	cerr << "PointCloud size after ground plane removal: " << processCloud->width * processCloud->height << " data points." << endl;
     //*/

    int maxPts = 50000;
    int minPts = 15;
	// double distThresholdCluster = 0.3;
	// double maxDiameter = 1;
	// segmenterSingleCut(minPts, maxPts, distThresholdCluster, maxDiameter);

	double thresholdDON = 0.25;
	double scaleLarge = scaleSmall*10;
    Segmenter segmenter(processCloud, &rawClusters);
    segmenter.loadDon(pathToPCDFile);
    segmenter.donSegmenter(minPts, maxPts, scaleSmall, scaleLarge, thresholdDON, true);


	double angleToVertical = 0.35;
    segmenter.stitchClusters(angleToVertical, maxDistanceStitches, stitchedClusters);

    // Pole detection followed by tree extraction based on rules
    RuleBasedDetection detector(&detectedPoles, &detectedTrees);
    detector.detectPoles(minPoleHeight, xyBoundThreshold, stitchedClusters);
    //detector.mergeExtractTrees(maxDistanceTrees);

    ioManager.writeDebugCloud(detectedPoles, "Poles.pcd");
}

void Master::buildRefClusters(string pathToPCDFile, double maxDistanceStitches, double scaleSmall){
	//ioManager.readDon(pathToPCDFile, _donCloud);
    int maxPts = 50000;
    int minPts = 15;
	double thresholdDON = 0.25;
	double scaleLarge = scaleSmall*10;
    Segmenter segmenter(processCloud, &rawClusters);
    segmenter.loadDon(pathToPCDFile);
    segmenter.donSegmenter(minPts, maxPts, scaleSmall, scaleLarge, thresholdDON, true);

    double angleToVertical = 0.35;
    segmenter.stitchClusters(angleToVertical, maxDistanceStitches, stitchedClusters);
	double minPoleHeight = 3;
	double xyBoundMin = 3;
	double xyBoundMax = 10;
    RuleBasedDetection detector(&detectedPoles, &detectedTrees);
    detector.detectTrees(minPoleHeight, xyBoundMin, xyBoundMax, stitchedClusters);

	ioManager.writeSegments(detectedTrees, "Trees");
}



void Master::runLandmarkClassifier(string pathToPCDFile, string pathToClassifier,
                                            double thresholdDON, int mode, bool rForest, double knnThreshold){
    /// Reading Input cloud
    //ioManager.readDon(pathToPCDFile, _donCloud);

    /* Uncomment to read raw pcd as input
    ioManager.readPCD(pathToPCDFile, processCloud);
    double meanKNoise = 10;
    double stdDevNoise = 1;

    cerr << "PointCloud size before noise removal: " << processCloud->width * processCloud->height << " data points." << endl;


    Preprocessor preprocessor(processCloud);
    preprocessor.outlierRemover(meanKNoise, stdDevNoise);

    cerr << "PointCloud size after noise removal: " << processCloud->width * processCloud->height << " data points." << endl;

    //*/

    ///* DON module
    /// DON thresholding and clustering. thresholdDON here is the DON cluster threshold
    int maxPts = 50000;
    int minPts = 15;
    double donScaleSmall = 0.5;
    double scaleLarge = donScaleSmall*10;
    Segmenter segmenter(processCloud, &rawClusters);
    segmenter.loadDon(pathToPCDFile);
    segmenter.donSegmenter(minPts, maxPts, donScaleSmall, scaleLarge, thresholdDON, true);
    //*/

    /* Normal clustering
    Segmenter segmenter(processCloud, rawClusters);
    segmenter.euclideanSegmenter(15, 50000, 0.35, 20);
    //*/

    /// Cluster Stitching
    double angleToVertical = 0.35;
    double maxDistanceStitches = 1.5;
    segmenter.stitchClusters(angleToVertical, maxDistanceStitches, stitchedClusters);
    rawClusters.clear();
    ioManager.writeDebugCloud(stitchedClusters, "stitches.pcd");

    /// Loading trained random forest classifier, extracting features and predicting
    FeatureBasedClassifier classifier(&poleLikeClusters, &detectedPoles, &detectedTrees);
    if (rForest)
        classifier.loadRandomForest(pathToClassifier);
    else
        classifier.loadKnn(pathToClassifier, knnThreshold);
    classifier.featureMatcher(stitchedClusters, mode, rForest);

    stitchedClusters.clear();
    ioManager.writeDebugCloud(poleLikeClusters, "pole-like.pcd");
    ioManager.writeDebugCloud(detectedPoles, "poles.pcd");
    ioManager.writeDebugCloud(detectedTrees, "trees.pcd");

}