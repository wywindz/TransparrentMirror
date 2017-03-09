#include <boost/make_shared.hpp>

#include <pcl/octree/octree_search.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>

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

std::size_t CVSFeature::getNumEdges()
{
    return edgeVectors.size();
}

const std::vector<CVSFeature> refineCVSFeatureList(const std::vector<CVSFeature> & cvsFeatureList)
{

}

// -------------- Class CVSEstimation ------------------
CVSEstimation::CVSEstimation() : radiusNeighbor(0.1), minDistance(0.2), minNumEdges(2)
{ }

CVSEstimation::~CVSEstimation()
{ }

void CVSEstimation::setInputCloud(const PointCloudConstPtr & ptrPointCloud)
{
    inputCloud = ptrPointCloud;
}

void CVSEstimation::setRadius(float radius)
{
    radiusNeighbor = radius;
}

void CVSEstimation::setMinNumEdges(std::size_t numEdges)
{
    minNumEdges = numEdges;
}

std::vector<CVSFeature> CVSEstimation::esimate()
{
    // Find corners.
    // Note: The output data type of HarrisKeypoint3D should contain intensity.
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> cornerDetector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr corners(new pcl::PointCloud<pcl::PointXYZI>());
    cornerDetector.setInputCloud(inputCloud);
    cornerDetector.setNonMaxSupression(true);
    cornerDetector.setRadius(0.1);
    cornerDetector.setThreshold(1e-3);
    cornerDetector.compute(*corners);

    // ToDo: May refine corners to reduce the computation, remove extra corners which are too close with each other.

    // Detect the CVS features.
    pcl::ExtractIndices<pcl::PointXYZ> extracter;
    extracter.setInputCloud(inputCloud);
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.1);
    octree.setInputCloud(inputCloud);
    octree.addPointsFromInputCloud();
    for (std::size_t i = 0; i < corners->size(); ++i) {
        // Extract the neighborhoods of these corners, in which edge detection will be performed.
        pcl::PointXYZ cornerPosition((*corners).points[i].x, (*corners).points[i].y, (*corners).points[i].z);
        std::vector<int> neighborIndices;
        std::vector<float> distances;
        octree.radiusSearch(cornerPosition, 1, neighborIndices, distances);

        pcl::PointCloud<pcl::PointXYZ>::Ptr keypointNeiborhoods(new pcl::PointCloud<pcl::PointXYZ>());
        // Need to convert the indices from std::vector<int> to boost::shared_ptr<std::vector<int> >.
        extracter.setIndices(boost::make_shared<std::vector<int> >(neighborIndices));
        extracter.filter(*keypointNeiborhoods);

        std::cout << keypointNeiborhoods->size() << std::endl;

        // ToDo:: Detect the edges in the neighborhood of the keypoint.
    }

    return std::vector<CVSFeature>();
}

float CVSEstimation::getRadius()
{
    return radiusNeighbor;
}

std::size_t CVSEstimation::getMinNumEdges()
{
    return minNumEdges;
}

} // namespace RADI
