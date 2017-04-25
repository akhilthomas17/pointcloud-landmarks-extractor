#ifndef COMMON_HPP_INCLUDED
#define COMMON_HPP_INCLUDED

// General include files:
#include <iostream>
#include <fstream>
#include <list>

using namespace std;

// PCL specific include files:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/esf.h>
#include <pcl/common/centroid.h>

class Cluster
{
public:

	Cluster(Eigen::Vector4f centroid_, double radius_, pcl::PointXYZ minPt_, pcl::PointXYZ maxPt_, pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud_):
	centroid(centroid_),
	radius(radius_),
	minPt(minPt_),
	maxPt(maxPt_),
	processed(false),
	clusterCloud(clusterCloud_){}

	Cluster(){
	clusterCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    }

	~Cluster(){}

	Eigen::Vector4f const& getCentroid(){return centroid;}
	double const& getRadius(){return radius;}
	double getZMin(){return minPt.z;}
	double getZMax(){return maxPt.z;}
	pcl::PointXYZ const& getMinPt(){return minPt;}
	pcl::PointXYZ const& getMaxPt(){return maxPt;}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getClusterCloud(){return clusterCloud;}
	bool const& isProcessed(){return processed;}
	void markProcessed(){processed = true;}

private:
	Eigen::Vector4f centroid;
	double radius;
	pcl::PointXYZ minPt;
	pcl::PointXYZ maxPt;
	bool processed;
	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud;

};


class Segment
{
public:
	Segment(){height = 0;}
	~Segment(){segmentParts.clear();};
	void mergeSegment(Segment& segment){
        segmentParts.insert(segmentParts.end(), segment.getSegmentParts().begin(), segment.getSegmentParts().end());
        double lenSelf = segmentParts.size();
        double lenSegment = segment.getSegmentParts().size();
        centroid = (segment.getCentroid() * lenSegment + centroid * lenSelf)/(lenSelf + lenSegment);
        minPt.x = min(minPt.x, segment.getMinPt().x);
        maxPt.x = max(maxPt.x, segment.getMaxPt().x);
        minPt.y = min(minPt.y, segment.getMinPt().y);
        maxPt.y = max(maxPt.y, segment.getMaxPt().y);
        minPt.z = min(minPt.z, segment.getMinPt().z);
        maxPt.z = max(maxPt.z, segment.getMaxPt().z);
        height = maxPt.z - minPt.z;
    }
	void addCluster(Cluster& cluster){
        segmentParts.push_back(cluster);
        int numClusters = segmentParts.size();
        centroid = (centroid * (numClusters-1) + cluster.getCentroid())/(numClusters);
        if (numClusters == 1){
            minPt = cluster.getMinPt();
            maxPt = cluster.getMaxPt();
            segmentCloud = cluster.getClusterCloud();
        }
        else{
            minPt.x = min(minPt.x, cluster.getMinPt().x);
            maxPt.x = max(maxPt.x, cluster.getMaxPt().x);
            minPt.y = min(minPt.y, cluster.getMinPt().y);
            maxPt.y = max(maxPt.y, cluster.getMaxPt().y);
            minPt.z = min(minPt.z, cluster.getMinPt().z);
            maxPt.z = max(maxPt.z, cluster.getMaxPt().z);
            *segmentCloud += *(cluster.getClusterCloud());
        }
        height = maxPt.z - minPt.z;
    }
	double getHeight(){return height;}
	double getZMax(){return maxPt.z;}
	double getZMin(){return minPt.z;}
	pcl::PointXYZ const& getMinPt(){return minPt;}
	pcl::PointXYZ const& getMaxPt(){return maxPt;}
	list<Cluster>& getSegmentParts(){return segmentParts;}
	Eigen::Vector4f getCentroid(){return centroid;}
	Eigen::Vector4f getBase(){return base;}
	pcl::PointCloud<pcl::PointXYZ>::Ptr getSegmentCloud(){return segmentCloud;}
	void setSegmentCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){segmentCloud = cloud;}
	void computeCentroid(){
		pcl::compute3DCentroid(*segmentCloud, centroid);
	}


private:
	list<Cluster> segmentParts;
	Eigen::Vector4f centroid;
	double height;
	pcl::PointXYZ minPt;
	pcl::PointXYZ maxPt;
	Eigen::Vector4f base;
	pcl::PointCloud<pcl::PointXYZ>::Ptr segmentCloud;

};


class Feature
{
public:
    Feature(){}
    ~Feature(){signatureAsVector.clear();}
    Feature& operator+=(const Feature feat){
        signatureAsVector.insert(signatureAsVector.end(),
                                       feat.signatureAsVector.begin(), feat.signatureAsVector.end());
        return  *this;
    }
    string name;
    vector<float> signatureAsVector;
};

// ** Helper functions!!!

static bool compareClusterHeight(Cluster& first, Cluster& second){
	return (first.getCentroid()[2] < second.getCentroid()[2]);
}

static void makeColoredPoint(pcl::PointXYZRGB & PtColored, pcl::PointXYZ const & Pt, uint32_t rgb){
	PtColored.x = Pt.x;
	PtColored.y = Pt.y;
	PtColored.z = Pt.z;
	PtColored.rgb = *reinterpret_cast<float*>(&rgb);
}

static void eigenV4f2PointXYZ(Eigen::Vector4f const &vec, pcl::PointXYZ &point){
	point.x = vec[0];
	point.y = vec[1];
	point.z = vec[2];
}

static void PointXYZ2eigenV4f2D(Eigen::Vector4f &vec, pcl::PointXYZ const &point){
	vec[0] = point.x;
	vec[1] = point.y;
	vec[2] = 1; // In order to check only x-y distance
	vec[3] = 0; // Check!!!!!!*************//////////********
}


#endif // COMMON_HPP_INCLUDED
