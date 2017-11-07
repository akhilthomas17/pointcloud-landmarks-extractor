# Efficient map representation by detecting and classifying Landmarks in 3D LiDAR scans

This project contains the code developed as part of my Master Project at Autonomous Intelligent Systems Lab at University of Freiburg.
The project was titled "Efficient map representation by detecting and classifying Landmarks in 3D LiDAR scans". Tayyab Naseer and Prof. Wolfram Burgard had been my advisors.

## Abstract

Detecting and classifying landmarks is a key task in a wide range of fields including simultaneous localization and mapping, urban modeling, surveillance etc. Landmarks of interest is specific to the application; trees and pole-like structures are often the favorites. Point cloud data gives more information about the underlying environment than images, and hence the problem of landmark detection and classification is intuitively easier using point clouds. In this research two novel functional pipelines for detecting and classifying landmarks, specifically pole-like objects and trees, from point clouds are designed. The belief that better clusters are a prerequisite for better classification is the prime motivation for these approaches. Both the designed tools use Difference of Normals based thresholding and segmentation. In the first approach, only pole-like objects are detected by employing few heuristics on the extracted clusters. However the second approach detects and classifies both pole-like objects and trees. In this approach, a combination of features - ensemble of shape functions and eigenvalue based features are extracted from the clusters. Later, a random forest multi-class classifier is trained to classify the clusters as pole-like objects, trees or neither of them. The designed pipeline is trained and tested using two point cloud maps of the Technical Faculty, University of Freiburg.

## Usage

The project contains the following executables:

pole_detector

> ./pole_detector [input mode] [path to pcd file] [DON cluster threshold]
[input mode]
1 : Input DON file
0 : Input raw PCD file

landmark_classifier

> ./landmark_classifier [input mode] [path to pcd file] [classifier mode] [path to classifier model] [DON cluster threshold]
[input mode]
1 : Input DON file
0 : Input raw PCD file
[classifier mode]
1 : Random forest classifier
0 : KNN

approach_landa

> ./approach_landa <path to pcd file> <num cuts> <min pole height>


train_knn
>./train_knn [model_directory] [options]
[options]
--eigVal : To use eigen value based features
--esf : To use esf based features
--combined : To use combination of above two

train_randomForest
> ./train_randomForest [model_directory] [options] [max_depth] [num_trees]
[options]
--eigVal : To use eigen value based features
--esf : To use esf based features
--combined : To use combination of above two
