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

private:
    pcl::PointXYZ corner;
    std::vector<Eigen::Vector3f> edgeVectors;
};

class CVSEstimation
{
public:
    CVSEstimation();
    ~CVSEstimation();

    void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr ptrPointCloud);
    void setRadius(float radius);
    void setMinNumEdges(std::size_t numEdges);
    std::vector<CVSFeature> esimate();

    float getRadius();
    std::size_t getMinNumEdges();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;
    float radiusRegion;
    float minNumEdges;
};

} // namespace RADI

#endif MIRROR_CVS_H_
