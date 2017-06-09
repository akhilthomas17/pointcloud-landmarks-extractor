#include <pole_detector.h>

PCLPoleDetector::PCLPoleDetector(){
	processCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	_donCloud = pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal>);
	stitchedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	poleLikeCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	poleCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	treeCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
}


PCLPoleDetector::~PCLPoleDetector(){
	rawClusters.clear();
	stitchedClusters.clear();
	poleLikeClusters.clear();
	detectedPoles.clear();
	detectedTrees.clear();
}


void PCLPoleDetector::writePCD(string pathToFile){
	pcl::PCDWriter writer;
	//*
	writer.writeBinary("stitches.pcd", *stitchedCloud);
  	std::cerr << "Saved " << stitchedCloud->points.size() << " data points to stitches.pcd." << std::endl;

  	writer.writeBinary("pole-like.pcd", *poleLikeCloud);
  	std::cerr << "Saved " << poleLikeCloud->points.size() << " data points to pole-like.pcd." << std::endl;


  	writer.writeBinary("poles.pcd", *poleCloud);
  	std::cerr << "Saved " << poleCloud->points.size() << " data points to poles.pcd." << std::endl;

  	writer.writeBinary("trees.pcd", *treeCloud);
  	std::cerr << "Saved " << treeCloud->points.size() << " data points to trees.pcd." << std::endl;

	//*/

	/*
  	writer.writeBinary("poles.pcd", *poleCloud);
	// pcl::io::savePCDFileASCII (pathToFile, *processCloud);
  	std::cerr << "Saved " << poleCloud->points.size() << " data points to poles.pcd." << std::endl;
	*/

}

void PCLPoleDetector::writePoles(){
	pcl::PCDWriter writer;
	int i = 0;
	for (std::list<Segment>::iterator it = detectedPoles.begin(); it != detectedPoles.end(); ++it){
		writer.writeBinary("clusters/cluster" + boost::lexical_cast<std::string>(i) + ".pcd", *(it->getSegmentCloud()));
		i += 1;
	}
}

void PCLPoleDetector::writeSegments(list<Segment>& segmentList){
    pcl::PCDWriter writer;
    int i = 0;
    for (std::list<Segment>::iterator it = segmentList.begin(); it != segmentList.end(); ++it){
        writer.writeBinary("segments/segment" + boost::lexical_cast<std::string>(i) + ".pcd", *(it->getSegmentCloud()));
        i += 1;
    }
}

void PCLPoleDetector::writeTrees(){
	pcl::PCDWriter writer;
	int i = 0;
	for (std::list<Segment>::iterator it = detectedTrees.begin(); it != detectedTrees.end(); ++it){
		writer.writeBinary("trees/tree" + boost::lexical_cast<std::string>(i) + ".pcd", *(it->getSegmentCloud()));
		i += 1;
	}
}



void PCLPoleDetector::poleDetector(double minPoleHeight, double xyBoundThreshold){
	boost::random::uniform_int_distribution<> dist(0, 255);
	for (list<Segment>::iterator it = stitchedClusters.begin(); it != stitchedClusters.end(); ++it){
		Segment candidatePole = *it;
		if (candidatePole.getHeight() > minPoleHeight){
			Eigen::Vector4f minVecX, maxVecX, minVecY, maxVecY;
			pcl::PointXYZ minPtX, maxPtX, minPtY, maxPtY;
			minPtY = minPtX = candidatePole.getMinPt();
			maxPtY = maxPtX = candidatePole.getMaxPt();
			minPtX.y = (minPtX.y + maxPtX.y)/2;
			maxPtX.y = minPtX.y;
			PointXYZ2eigenV4f2D(minVecX, minPtX);
	    	PointXYZ2eigenV4f2D(maxVecX, maxPtX);
			minPtY.x = (minPtY.x + maxPtY.x)/2;
			maxPtY.x = minPtY.x;
			PointXYZ2eigenV4f2D(minVecY, minPtY);
	    	PointXYZ2eigenV4f2D(maxVecY, maxPtY);

	    	double xyBound = std::max((maxVecX - minVecX).norm(), (maxVecY - minVecY).norm());
	    	if (xyBound <= xyBoundThreshold){
				detectedPoles.push_back(candidatePole);
				/* Uncomment to plot single cloud
				//** Making single color for each detected pole
				uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
				uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				for (list<Cluster>::iterator it = candidatePole.getSegmentParts().begin(); it != candidatePole.getSegmentParts().end(); ++it){
					for (size_t i = 0; i < it->getClusterCloud()->points.size(); ++i){
						pcl::PointXYZRGB PtColored;
	  					makeColoredPoint(PtColored, it->getClusterCloud()->points[i], rgb);
	  					poleCloud->points.push_back(PtColored);
					}
				}
				//*/
			}
		}
	}
	cerr << "Number of detected pole like structures: " << detectedPoles.size() << endl;
}


void PCLPoleDetector::treeExtractor(double minTreeHeight, double xyBoundMin, double xyBoundMax){
	boost::random::uniform_int_distribution<> dist(0, 255);
	for (list<Segment>::iterator it = stitchedClusters.begin(); it != stitchedClusters.end(); ++it){
		Segment candidateTree = *it;
		if (candidateTree.getHeight() > minTreeHeight){
			Eigen::Vector4f minVecX, maxVecX, minVecY, maxVecY;
			pcl::PointXYZ minPtX, maxPtX, minPtY, maxPtY;
			minPtY = minPtX = candidateTree.getMinPt();
			maxPtY = maxPtX = candidateTree.getMaxPt();
			minPtX.y = (minPtX.y + maxPtX.y)/2;
			maxPtX.y = minPtX.y;
			PointXYZ2eigenV4f2D(minVecX, minPtX);
	    	PointXYZ2eigenV4f2D(maxVecX, maxPtX);
			minPtY.x = (minPtY.x + maxPtY.x)/2;
			maxPtY.x = minPtY.x;
			PointXYZ2eigenV4f2D(minVecY, minPtY);
	    	PointXYZ2eigenV4f2D(maxVecY, maxPtY);

	    	double xyBound = std::max((maxVecX - minVecX).norm(), (maxVecY - minVecY).norm());
	    	if (xyBound <= xyBoundMax && xyBound >= xyBoundMin){
				detectedTrees.push_back(candidateTree);
				/* Uncomment to plot single cloud
				//** Making single color for each detected pole
				uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
				uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				for (list<Cluster>::iterator it = candidateTree.getSegmentParts().begin(); it != candidateTree.getSegmentParts().end(); ++it){
					for (size_t i = 0; i < it->getClusterCloud()->points.size(); ++i){
						pcl::PointXYZRGB PtColored;
	  					makeColoredPoint(PtColored, it->getClusterCloud()->points[i], rgb);
	  					poleCloud->points.push_back(PtColored);
					}
				}
				//*/
			}
		}
	}
	cerr << "Number of detected tree like structures: " << detectedTrees.size() << endl;
}


void PCLPoleDetector::treeExtractor(double maxDistanceTrees){
	boost::random::uniform_int_distribution<> dist(0, 255);
	list<Segment>::iterator it, it2;
	it = detectedPoles.begin();
	int j = 1;
	while(it != detectedPoles.end()){
		Segment candidateTree = *it;
		bool tree = false;
		it2 = detectedPoles.begin();
		advance(it2, j);
		while (it2!= detectedPoles.end()){
			double diffDistance = (candidateTree.getCentroid() - it2->getCentroid()).norm();
			if (diffDistance < maxDistanceTrees){
				tree = true;
				candidateTree.mergeSegment(*it2);
				it2 = detectedPoles.erase(it2);
			}
			else
				++it2;
		}
		if (tree){
			detectedTrees.push_back(candidateTree);
			/* Uncomment to plot trees
			//** Making single color for each detected pole
			uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
			uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
			for (list<Cluster>::iterator it3 = candidateTree.getSegmentParts().begin(); it3 != candidateTree.getSegmentParts().end(); ++it3){
				for (size_t i = 0; i < it3->getClusterCloud()->points.size(); ++i){
					pcl::PointXYZRGB PtColored;
	  				makeColoredPoint(PtColored, it3->getClusterCloud()->points[i], rgb);
	  				stitchedCloud->points.push_back(PtColored);
				}
			}
			//*/
			it = detectedPoles.erase(it);
		}
		else{
			/* Uncomment to plot poles
			//** Making single color for each detected pole
			uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
			uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
			for (list<Cluster>::iterator it3 = candidateTree.getSegmentParts().begin(); it3 != candidateTree.getSegmentParts().end(); ++it3){
				for (size_t i = 0; i < it3->getClusterCloud()->points.size(); ++i){
					pcl::PointXYZRGB PtColored;
	  				makeColoredPoint(PtColored, it3->getClusterCloud()->points[i], rgb);
	  				poleCloud->points.push_back(PtColored);
				}
			}
			//*/
			++it;
			++j;
		}
	}
	cerr << "Number of detected Poles: " << detectedPoles.size() << endl;
	cerr << "Number of detected Trees: " << detectedTrees.size() << endl;
}

void PCLPoleDetector::clusterFilter(double minHeight, double xyBoundThreshold){
	boost::random::uniform_int_distribution<> dist(0, 255);
	for (list<Segment>::iterator it = stitchedClusters.begin(); it != stitchedClusters.end(); ++it){
		Segment candidatePole = *it;
		if (candidatePole.getHeight() > minHeight){
			Eigen::Vector4f minVecX, maxVecX, minVecY, maxVecY;
			pcl::PointXYZ minPtX, maxPtX, minPtY, maxPtY;
			minPtY = minPtX = candidatePole.getMinPt();
			maxPtY = maxPtX = candidatePole.getMaxPt();
			minPtX.y = (minPtX.y + maxPtX.y)/2;
			maxPtX.y = minPtX.y;
			PointXYZ2eigenV4f2D(minVecX, minPtX);
	    	PointXYZ2eigenV4f2D(maxVecX, maxPtX);
			minPtY.x = (minPtY.x + maxPtY.x)/2;
			maxPtY.x = minPtY.x;
			PointXYZ2eigenV4f2D(minVecY, minPtY);
	    	PointXYZ2eigenV4f2D(maxVecY, maxPtY);

	    	double xyBound = std::max((maxVecX - minVecX).norm(), (maxVecY - minVecY).norm());
	    	if (xyBound <= xyBoundThreshold){
				poleLikeClusters.push_back(candidatePole);
				//* Uncomment to plot single cloud
				//** Making single color for each detected pole
				uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
				uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				for (list<Cluster>::iterator it2 = candidatePole.getSegmentParts().begin(); it2 != candidatePole.getSegmentParts().end(); ++it2){
					for (size_t i = 0; i < it2->getClusterCloud()->points.size(); ++i){
						pcl::PointXYZRGB PtColored;
	  					makeColoredPoint(PtColored, it2->getClusterCloud()->points[i], rgb);
	  					poleLikeCloud->points.push_back(PtColored);
					}
				}
				//*/
			}
		}
	}
	cerr << "Number of detected pole-like structures: " << poleLikeClusters.size() << endl;
}

void PCLPoleDetector::algorithmSingleCut(string pathToPCDFile, double xyBoundThreshold,
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
	poleDetector(minPoleHeight, xyBoundThreshold);
	//treeExtractor(maxDistanceTrees);

	writePCD("output_pcd.pcd");
	/*
	while (!viewer->wasStopped ()) { // Display the visualiser until 'q' key is pressed
    	viewer->spinOnce (100);
    	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	*/
}

void PCLPoleDetector::buildRefClusters(string pathToPCDFile, double maxDistanceStitches, double scaleSmall){
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
	treeExtractor(minPoleHeight, xyBoundMin, xyBoundMax);

	writeTrees();
}

void PCLPoleDetector::loadEsfData(vector<string>& labelList, string name_file){
    // Reading labels from training file
  	ifstream fs;
  	fs.open (name_file.c_str ());
	string line;
	while (!fs.eof ()){
		getline (fs, line);
		if (line.empty ())
		  continue;
		labelList.push_back(line);
	}
	fs.close ();
	//cerr << "ESF data loaded, size of data: " << labelList.size() << endl;
}

bool PCLPoleDetector::isPole(flann::Matrix<int> const& k_indices, vector<string> const& labelList){
	string label_nn = labelList.at(k_indices[0][0]);
	if (label_nn.find("Tree") != -1){
		return(false);
	}
	return (true);
}

void PCLPoleDetector::knnFinder(flann::Index<flann::ChiSquareDistance<float> > const &kdTree,
                                vector<string> const &labelList, double kdTreeThreshold, int mode){
	boost::random::uniform_int_distribution<> dist(0, 255);
	FeatureDescriptor descriptor;
	for (list<Segment>::iterator it = poleLikeClusters.begin(); it != poleLikeClusters.end(); ++it){
		Segment candidate = *it;
        Feature feature;
        switch (mode){
            case 0:
                descriptor.describeEsfFeature(candidate.getSegmentCloud(),&feature);
                break;
            case 1:
                descriptor.describeEigenFeature(&candidate,&feature);
                break;
            case 2: {
                descriptor.describeEsfFeature(candidate.getSegmentCloud(),&feature);
                Feature eigVal;
                descriptor.describeEigenFeature(&candidate,&eigVal);
                feature+=eigVal;
                break;
            }
        }
		flann::Matrix<float> feature_flann = flann::Matrix<float>(new float[feature.signatureAsVector.size ()],
                                                           1, feature.signatureAsVector.size ());
  		memcpy (&feature_flann.ptr ()[0], &feature.signatureAsVector[0],
                feature_flann.cols * feature_flann.rows * sizeof (float));

  		// Calculating k nearest neighbours (k = 1)
  		int k = 1;
  		flann::Matrix<int> k_indices = flann::Matrix<int>(new int[k], 1, k);
  		flann::Matrix<float> k_distances = flann::Matrix<float>(new float[k], 1, k);
  		kdTree.knnSearch (feature_flann, k_indices, k_distances, k, flann::SearchParams (512));
  		delete[] feature_flann.ptr ();

  		// Checking if the mean distance is less than threshold
  		double meanDist = k_distances[0][0];
  		if( meanDist < kdTreeThreshold){
  			// Check if the cluster is pole. Else, it is detected as a tree
  			if(isPole(k_indices, labelList)){
  				detectedPoles.push_back(candidate);
  				//* Uncomment to plot single cloud
				//** Making single color for each detected pole
				uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
				uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				for (list<Cluster>::iterator it2 = candidate.getSegmentParts().begin(); it2 != candidate.getSegmentParts().end(); ++it2){
					for (size_t i = 0; i < it2->getClusterCloud()->points.size(); ++i){
						pcl::PointXYZRGB PtColored;
	  					makeColoredPoint(PtColored, it2->getClusterCloud()->points[i], rgb);
	  					poleCloud->points.push_back(PtColored);
					}
				}
				//*/
  			}else {
  				detectedTrees.push_back(candidate);
  				//* Uncomment to plot single cloud
				//** Making single color for each detected pole
				uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
				uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				for (list<Cluster>::iterator it2 = candidate.getSegmentParts().begin(); it2 != candidate.getSegmentParts().end(); ++it2){
					for (size_t i = 0; i < it2->getClusterCloud()->points.size(); ++i){
						pcl::PointXYZRGB PtColored;
	  					makeColoredPoint(PtColored, it2->getClusterCloud()->points[i], rgb);
	  					treeCloud->points.push_back(PtColored);
					}
				}
				//*/
  			}
  		}
	}
	cerr << "Number of detected Poles: " << detectedPoles.size() << endl;
	cerr << "Number of detected Trees: " << detectedTrees.size() << endl;

}

void PCLPoleDetector::algorithmFeatureDescriptorBased(string pathToPCDFile, string pathToDataFolder,
													  double donScaleSmall, double kdTreeThreshold, int mode){
	/// Reading Input cloud
	//ioManager.readDon(pathToPCDFile, _donCloud);

	/// DON thresholding followed by Euclidean clustering. ScaleSmall here is the Cluster Threshold
    int maxPts = 50000;
    int minPts = 15;
	double thresholdDON = 0.25;
	double scaleLarge = donScaleSmall*10;
    Segmenter segmenter(processCloud, &rawClusters);
    segmenter.loadDon(pathToPCDFile);
    segmenter.donSegmenter(minPts, maxPts, donScaleSmall, scaleLarge, thresholdDON, true);


    /// Cluster Stitching
	double angleToVertical = 0.35;
	double maxDistanceStitches = 1.5;
    segmenter.stitchClusters(angleToVertical, maxDistanceStitches, stitchedClusters);
	rawClusters.clear();

	/// Cluster filters based on "minimum height" and "maximum xy-bound"
	double minHeight = 3;
	double xyBoundThreshold = 20;
	clusterFilter(minHeight, xyBoundThreshold);
	stitchedClusters.clear();

	/// Loading trained KdTree
	string kdtree_file = pathToDataFolder + "kdtree.idx";
	string training_data_h5_file = pathToDataFolder + "training_data.h5";
	string training_data_name_file = pathToDataFolder + "training_data.list";
	vector<string> labelList;
	loadEsfData(labelList, training_data_name_file);
	// Reading Kd Tree from training file
	flann::Matrix<float> data;
	flann::load_from_file (data, training_data_h5_file, "training_data");
	flann::Index<flann::ChiSquareDistance<float> > kdTree (data, flann::SavedIndexParams (kdtree_file));
    kdTree.buildIndex ();

    /// Finding Feature signature for each of the clusters, followed by matching with trained data
    knnFinder(kdTree, labelList, kdTreeThreshold, mode);


	writePCD("output.pcd");
}

void PCLPoleDetector::featureMatcher(RandomForestLearner& classifier, int mode){
    boost::random::uniform_int_distribution<> dist(0, 255);
    FeatureDescriptor descriptor;
    for (list<Segment>::iterator it = poleLikeClusters.begin(); it != poleLikeClusters.end(); ++it){
        Segment candidate = *it;
        Feature feature;
        switch (mode){
            case 0:
                /*
                for (int i = 0; i < 9; ++i) {
                    Feature temp_feature;
                    descriptor.describeEsfFeature(candidate.getSegmentCloud(),&temp_feature);
                    string predictedClass;
                    classifier.predictClass(temp_feature, predictedClass);
                    cerr << predictedClass << endl;
                }
                 //*/
                descriptor.describeEsfFeature(candidate.getSegmentCloud(),&feature);
                break;
            case 1:
                descriptor.describeEigenFeature(&candidate,&feature);
                break;
            case 2: {
                descriptor.describeEsfFeature(candidate.getSegmentCloud(),&feature);
                Feature eigVal;
                descriptor.describeEigenFeature(&candidate,&eigVal);
                feature+=eigVal;
                break;
            }
        }
        // Predicting the label using classifier
        string predictedClass;
        classifier.predictClass(feature, predictedClass);
        //cerr << predictedClass << endl;
        //cerr << "-------" << endl;

        // Check if the cluster is pole. Else, it is detected as a tree
        if(predictedClass.find("Pole") != -1){
            detectedPoles.push_back(candidate);
            //* Uncomment to plot single cloud
            //** Making single color for each detected pole
            uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            for (list<Cluster>::iterator it2 = candidate.getSegmentParts().begin(); it2 != candidate.getSegmentParts().end(); ++it2){
                for (size_t i = 0; i < it2->getClusterCloud()->points.size(); ++i){
                    pcl::PointXYZRGB PtColored;
                    makeColoredPoint(PtColored, it2->getClusterCloud()->points[i], rgb);
                    poleCloud->points.push_back(PtColored);
                }
            }
            //*/
        }else if (predictedClass.find("Tree") != -1){
            detectedTrees.push_back(candidate);
            //* Uncomment to plot single cloud
            //** Making single color for each detected pole
            uint8_t r = dist(randomGen), g = dist(randomGen), b = dist(randomGen);
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            for (list<Cluster>::iterator it2 = candidate.getSegmentParts().begin(); it2 != candidate.getSegmentParts().end(); ++it2){
                for (size_t i = 0; i < it2->getClusterCloud()->points.size(); ++i){
                    pcl::PointXYZRGB PtColored;
                    makeColoredPoint(PtColored, it2->getClusterCloud()->points[i], rgb);
                    treeCloud->points.push_back(PtColored);
                }
            }
            //*/
        }

    }
    cerr << "Number of detected Poles: " << detectedPoles.size() << endl;
    cerr << "Number of detected Trees: " << detectedTrees.size() << endl;
}

void PCLPoleDetector::algorithmClassifierBased(string pathToPCDFile, string pathToClassifier,
                                               double thresholdDON, int mode){
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

    /// Cluster filters based on "minimum height" and "maximum xy-bound"
    double minHeight = 2;
    double xyBoundThreshold = 20;
    clusterFilter(minHeight, xyBoundThreshold);
    stitchedClusters.clear();

	//writeSegments(poleLikeClusters);

    // Loading Random Forest Classifier
    RandomForestLearner classifier;
    classifier.loadClassifier(pathToClassifier);
    featureMatcher(classifier, mode);

    writePCD("output.pcd");

}