#ifndef FEATUREEXTRACTOR_HPP
#define FEATUREEXTRACTOR_HPP

#include <Libs/Structures.hpp>
#include <pcl/common/common.h>


class FeatureExtractor
{
public:
    /** Default constructor */
    FeatureExtractor();
    /** Default destructor */
    virtual ~FeatureExtractor();
    bool filterSegment(Segment* candidate, double minHeight, double xyBoundThreshold);
    void extractEsfFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud, Feature *esfFeature);
    void extractEigenFeature(Segment *candidate, Feature *eigenFeature);
protected:
private:
};

#endif // FEATUREDESCRIPTOR_H
