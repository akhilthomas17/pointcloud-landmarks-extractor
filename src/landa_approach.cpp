//
// Created by akhil on 19.07.17.
//

#include <Preprocessor/Preprocessor.hpp>
#include <Segmenter/Segmenter.hpp>
#include <Detector/RuleBasedDetector.hpp>

void runLandaApproach(string pathToPCDFile, int numCuts, double xyBoundThreshold,
                      double maxDistanceStitches, double minPoleHeight){
    IO ioManager;
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    list<Cluster> rawClusters;
    list<Segment> stitchedClusters;
    list<Segment> detectedPoles;
    list<Segment> detectedTrees;

    ioManager.readPCD(pathToPCDFile, processCloud);
    int meanKNoise = 10;
    double stdDevNoise = 1;

    cerr << "PointCloud size before noise removal: " << processCloud->width * processCloud->height << " data points." << endl;

    Preprocessor preprocessor(processCloud);
    preprocessor.outlierRemover(meanKNoise, stdDevNoise);
    cerr << "PointCloud size after noise removal: " << processCloud->width * processCloud->height << " data points." << endl;

    // Ground plane removal
    double distThreshold = 0.2;
    preprocessor.groundPlaneRemover(distThreshold);
    Segmenter segmenter(processCloud, &rawClusters);

    // Horizontal cut based segmentation
    pcl::PointXYZ minPt , maxPt;
    pcl::getMinMax3D(*processCloud, minPt, maxPt);
    double minHeight = minPt.z;
    double maxHeight = maxPt.z;
    double stepCut = (maxHeight - minHeight)/numCuts;


    for (double minCut = minHeight; minCut < maxHeight; minCut+=stepCut) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cutCloud(new pcl::PointCloud<pcl::PointXYZ>);
        list<Cluster> cutClusters;
        preprocessor.cutCloudBuilder(cutCloud, minCut, minCut+stepCut);
        // Euclidean clustering
        Segmenter cutSegmenter(cutCloud, &cutClusters);
        cutSegmenter.euclideanSegmenter(15, 50000, 0.35, 20);
        rawClusters.merge(cutClusters, compareClusterHeight);
    }

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

int main(int argc, char const *argv[])
{
    if ( argc != 4 ) // argc should be 4 for correct execution
    {
        cout<<"usage: "<< argv[0] <<" <path to pcd file> <num cuts> <min pole height>\n";
    }

    else {
        double xyBoundThreshold = 1;
        double maxStitchDistance = 1.5;
        runLandaApproach(argv[1], atoi(argv[2]), xyBoundThreshold, maxStitchDistance, atof(argv[3]));
    }

    return 0;
}
