#ifndef FEATUREDESCRIPTOR_H
#define FEATUREDESCRIPTOR_H

// General include files:
#include <iostream>

using namespace std;

// PCL specific include files:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/esf.h>

class Feature
{
public:
    Feature();
    ~Feature(){signatureAsVector.clear();}
    string name;
    vector<float> signatureAsVector;
};


class FeatureDescriptor
{
    public:
        /** Default constructor */
        FeatureDescriptor();
        /** Default destructor */
        virtual ~FeatureDescriptor();
        void describeEsfFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud, Feature* esfFeature);
        void describeEigenFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud, Feature* eigenFeature);
    protected:
    private:
};

#endif // FEATUREDESCRIPTOR_H
