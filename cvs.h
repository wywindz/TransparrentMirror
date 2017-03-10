/*
 * Define Corner Vector Sequence (CVS) Feature as well as the related classes for estimating the feature.
 *
 */

#ifndef MIRROR_CVS_H_
#define MIRROR_CVS_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

namespace RADI {

class CVSFeature
{
public:
    CVSFeature(pcl::PointXYZ cornerPosition = pcl::PointXYZ());
    CVSFeature(pcl::PointXYZ cornerPosition, std::vector<Eigen::Vector3f> vectors);
    ~CVSFeature();

    template<typename PointType>
    void setCorner(PointType cornerPosition)
    {
        corner = pcl::PointXYZ(cornerPosition[0], cornerPosition[1], cornerPosition[2]);
    }

    const pcl::PointXYZ getCorner();

    void appendVector(const Eigen::Vector3f & vector);
    const std::vector<Eigen::Vector3f> & getVectors() const;

    std::vector<float> getIncludedAngles();
    std::size_t getNumEdges();

private:
    pcl::PointXYZ corner;
    std::vector<Eigen::Vector3f> edgeVectors;
};

// Refine CVS feature list, for example, remove extra features which are two close with each other or remove features
// which have smaller edge number than the threshold.
const std::vector<CVSFeature> refineCVSFeatureList(const std::vector<CVSFeature> & cvsFeatureList);

class Edge
{
public:
    Eigen::Vector3f point;
    Eigen::Vector3f orientVector;
};

class EdgeDetector
{
public:
    EdgeDetector();
    ~EdgeDetector();

    // typedefs
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

    void setInputCloud(const PointCloudConstPtr & ptrPointCloud);
    void setRadius(float radius);
    void setKPoints(std::size_t kPoints);
    void compute(std::vector<Edge> & edgeList);

    float getRadius();
    std::size_t getKPoints();

private:
    PointCloudConstPtr inputCloud;
    float radiusNeighbor;
    float numNeighborPoints;
};

class CVSEstimation
{
public:
    CVSEstimation();
    ~CVSEstimation();

    // typedefs
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

    void setInputCloud(const PointCloudConstPtr & ptrPointCloud);
    void setRadius(float radius);
    void setMinDistance(float distance);
    void setMinNumEdges(std::size_t numEdges);
    std::vector<CVSFeature> esimate();

    float getRadius();
    float getMinDistance();
    std::size_t getMinNumEdges();

private:
    // pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud;
    PointCloudConstPtr inputCloud;
    float radiusNeighbor;
    float minDistance;
    float minNumEdges;
};

} // namespace RADI

#endif // MIRROR_CVS_H_
