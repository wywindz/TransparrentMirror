#include "cvs.h"

namespace RADI {

// -------------- Class CVSFeature ------------------
CVSFeature::CVSFeature(pcl::PointXYZ cornerPosition)
        : corner(cornerPosition), edgeVectors(std::vector<Eigen::Vector3f>())
{ }

CVSFeature::CVSFeature(pcl::PointXYZ cornerPosition, std::vector<Eigen::Vector3f> vectors)
        : corner(cornerPosition), edgeVectors(vectors)
{ }

CVSFeature::~CVSFeature()
{ }

const pcl::PointXYZ CVSFeature::getCorner()
{
    return corner;
}

void CVSFeature::appendVector(const Eigen::Vector3f &vector)
{
    edgeVectors.push_back(vector);
}

const std::vector<Eigen::Vector3f> & CVSFeature::getVectors() const
{
    return edgeVectors;
}

std::vector<float> CVSFeature::getIncludedAngles()
{
    // ToDO: Calculate the included angles between each pair of edge vectors.
}

// -------------- Class CVSEstimation ------------------
CVSEstimation::CVSEstimation()
{ }

CVSEstimation::~CVSEstimation()
{ }

void CVSEstimation::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr ptrPointCloud)
{
    inputCloud = ptrPointCloud;
}

void CVSEstimation::setRadius(float radius)
{
    radiusRegion = radius;
}

void CVSEstimation::setMinNumEdges(std::size_t numEdges)
{
    minNumEdges = numEdges;
}

std::vector<CVSFeature> CVSEstimation::esimate()
{
    // ToDo: Implement the estimation.
}

float CVSEstimation::getRadius()
{
    return radiusRegion;
}

std::size_t CVSEstimation::getMinNumEdges()
{
    return minNumEdges;
}

} // namespace RADI
