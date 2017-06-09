#ifndef FEATUREDESCRIPTOR_H
#define FEATUREDESCRIPTOR_H

#include <Structures.hpp>
#include <pcl/common/common.h>


class FeatureDescriptor
{
    public:
        /** Default constructor */
        FeatureDescriptor();
        /** Default destructor */
        virtual ~FeatureDescriptor();
        void describeEsfFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud, Feature* esfFeature);
        void describeEigenFeature(Segment* segment, Feature* eigenFeature);
    protected:
    private:
};

#endif // FEATUREDESCRIPTOR_H
