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

/**
 * Initial pole detector using euclidean clustering
 * @param pathToPCDFile
 * @param xyBoundThreshold
 * @param maxDistanceStitches
 * @param minPoleHeight
 */
void Master::runInitialApproach(string pathToPCDFile, double xyBoundThreshold, double maxDistanceStitches,
                                double minPoleHeight) {
    ioManager.readPCD(pathToPCDFile, processCloud);

    cerr << "PointCloud size before noise removal: " << processCloud->width * processCloud->height
         << " data points." << endl;

    Preprocessor preprocessor(processCloud);

    // Outlier removal
    int meanKNoise = 10;
    double stdDevNoise = 1;
    preprocessor.outlierRemover(meanKNoise, stdDevNoise);

    // Ground plane removal
    double distThreshold = 0.2;
    preprocessor.groundPlaneRemover(distThreshold);

    // Euclidean clustering
    Segmenter segmenter(processCloud, &rawClusters);
    segmenter.euclideanSegmenter(15, 50000, 0.35, 20);
    /// Cluster Stitching
    double angleToVertical = 0.35;
    segmenter.stitchClusters(angleToVertical, maxDistanceStitches, stitchedClusters);
    rawClusters.clear();
    ioManager.writeDebugCloud(stitchedClusters, "stitches.pcd");

    // Pole detection followed by tree extraction based on rules
    RuleBasedDetection detector(&detectedPoles, &detectedTrees);
    detector.detectPoles(minPoleHeight, xyBoundThreshold, stitchedClusters);

    ioManager.writeDebugCloud(detectedPoles, "Poles.pcd");
}


/**
 * runPoleDetector runs the pole detector algorithm with DON clustering
 * @param donInput true if input is DON calculated PCD
 * @param pathToPCDFile path to input point cloud
 * @param donThreshold cluster threshold for DON
 * @param maxDistanceStitches
 * @param xyBoundThreshold
 * @param minPoleHeight
 */
void Master::runPoleDetector(bool donInput, string pathToPCDFile, double donThreshold, double maxDistanceStitches,
                             double xyBoundThreshold, double minPoleHeight) {
    if(!donInput){
        /// Reading Input cloud
        ioManager.readPCD(pathToPCDFile, processCloud);
        int meanKNoise = 10;
        double stdDevNoise = 1;

        cerr << "PointCloud size before noise removal: " << processCloud->width * processCloud->height << " data points." << endl;

        Preprocessor preprocessor(processCloud);
        preprocessor.outlierRemover(meanKNoise, stdDevNoise);
        cerr << "PointCloud size after noise removal: " << processCloud->width * processCloud->height << " data points." << endl;
    }

    ///* DON module
    Segmenter segmenter(processCloud, &rawClusters);
    /// DON thresholding and clustering. thresholdDON here is the DON cluster threshold
    int maxPts = 50000;
    int minPts = 15;
    double donScaleSmall = 0.5;
    double scaleLarge = donScaleSmall*10;
    if(donInput)
        segmenter.loadDon(pathToPCDFile);
    segmenter.donSegmenter(minPts, maxPts, donScaleSmall, scaleLarge, donThreshold, donInput);
    //*/

    /// Cluster Stitching
    double angleToVertical = 0.35;
    segmenter.stitchClusters(angleToVertical, maxDistanceStitches, stitchedClusters);
    rawClusters.clear();
    ioManager.writeDebugCloud(stitchedClusters, "stitches.pcd");

    // Pole detection followed by tree extraction based on rules
    RuleBasedDetection detector(&detectedPoles, &detectedTrees);
    detector.detectPoles(minPoleHeight, xyBoundThreshold, stitchedClusters);

    // Uncomment to include tree extraction
    //detector.mergeExtractTrees(maxDistanceTrees);

    ioManager.writeDebugCloud(detectedPoles, "Poles.pcd");
}


/**
 * buildRefClusters to generate reference clusters for training random forest
 * @param pathToPCDFile path to input pcd file
 * @param maxDistanceStitches
 * @param scaleSmall DON small radii
 */
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


/**
 * runLandmarkClassifier to run the landmark classifier algorithm on input point cloud
 * @param donInput true if input is DON calculated PCD
 * @param pathToPCDFile the path to input point cloud
 * @param thresholdDON cluster threshold for DON
 * @param featureMode the feature descriptor mode used
 * @param rForest true if random forest is the classifier
 * @param pathToClassifier the path to trained classifier
 * @param knnThreshold KNN threshold if KNN is used for classification
 */
void Master::runLandmarkClassifier(bool donInput, string pathToPCDFile, double smallRadiusDON, double thresholdDON,
                                   int featureMode, bool rForest, string pathToClassifier, double knnThreshold) {
    if(!donInput){
        /// Reading Input cloud
        ioManager.readPCD(pathToPCDFile, processCloud);
        int meanKNoise = 10;
        double stdDevNoise = 1;

        cerr << "PointCloud size before noise removal: " << processCloud->width * processCloud->height << " data points." << endl;

        Preprocessor preprocessor(processCloud);
        preprocessor.outlierRemover(meanKNoise, stdDevNoise);
        cerr << "PointCloud size after noise removal: " << processCloud->width * processCloud->height << " data points." << endl;
    }

    ///* DON module
    Segmenter segmenter(processCloud, &rawClusters);
    /// DON thresholding and clustering. thresholdDON here is the DON cluster threshold
    int maxPts = 50000;
    int minPts = 15;
    double largeRadius = smallRadiusDON*10;
    if(donInput)
        segmenter.loadDon(pathToPCDFile);
    segmenter.donSegmenter(minPts, maxPts, smallRadiusDON, largeRadius, thresholdDON, donInput);
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
    classifier.featureMatcher(stitchedClusters, featureMode, rForest);

    stitchedClusters.clear();
    ioManager.writeDebugCloud(poleLikeClusters, "pole-like.pcd");
    ioManager.writeDebugCloud(detectedPoles, "poles.pcd");
    ioManager.writeDebugCloud(detectedTrees, "trees.pcd");
}


