#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>

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

// -------------- Class EdgeDetector ------------------
EdgeDetector::EdgeDetector()
{ }

EdgeDetector::~EdgeDetector()
{ }

void EdgeDetector::setInputCloud(const PointCloudConstPtr & ptrPointCloud)
{
    inputCloud = ptrPointCloud;
}

void EdgeDetector::setRadius(float radius)
{
    radiusNeighbor = radius;
}

void EdgeDetector::setKPoints(std::size_t kPoints)
{
    numNeighborPoints = kPoints;
}

void EdgeDetector::compute(std::vector<Edge> & edgeList)
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.01);
    octree.setInputCloud(inputCloud);
    for (std::size_t iPoint = 0; iPoint < inputCloud->size(); ++iPoint) {
        const pcl::PointXYZ & point = (*inputCloud)[iPoint];
        // Search 'K' points around the 'point'.
        std::vector<int> indices;
        std::vector<float> distances;
        octree.nearestKSearch(point, numNeighborPoints, indices, distances);

        if (indices.size()) {
            // Calculate the center location of the neighborhood.
            Eigen::Vector3f center;
            for (std::size_t iNeighbor = 0; iNeighbor < indices.size(); ++iNeighbor) {
                Eigen::Vector3f pointQ = Eigen::Vector3f((*inputCloud)[indices[iNeighbor]].x,
                        (*inputCloud)[indices[iNeighbor]].y, (*inputCloud)[indices[iNeighbor]].z);
                center += pointQ;
            }
            center /= indices.size();

            // Calculate correlation matrix.
            Eigen::Matrix3f matCorrelation;
            for (std::size_t iNeighbor = 0; iNeighbor < indices.size(); ++iNeighbor) {
                Eigen::Vector3f pointQ = Eigen::Vector3f((*inputCloud)[indices[iNeighbor]].x,
                        (*inputCloud)[indices[iNeighbor]].y, (*inputCloud)[indices[iNeighbor]].z);
                Eigen::Vector3f vectCQ = pointQ - center;
                matCorrelation += vectCQ * Eigen::Transpose<Eigen::Vector3f>(vectCQ);
            }

            // Calculate eigen values and eigen vectors of the correlation matrix.
            Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver;
            eigenSolver.compute(matCorrelation);
            Eigen::Vector3f eigenValues = eigenSolver.eigenvalues().real();
            std::vector<Eigen::Vector3f> eigenVectors(3);
            eigenVectors[0] = eigenSolver.eigenvectors().col(0).real();
            eigenVectors[1] = eigenSolver.eigenvectors().col(1).real();
            eigenVectors[2] = eigenSolver.eigenvectors().col(2).real();

            // Sort the eigen values.
            std::vector<std::size_t> orderIndices(3);
            std::size_t idxN = 0;
            std::generate(eigenValues.data(), eigenValues.data()+eigenValues.size(), [&]{ return idxN++; });
            std::sort(orderIndices.begin(), orderIndices.end(),
                      [&](int idx_1, int idx_2){ return eigenValues[idx_1] < eigenValues[idx_2]; });
            float lambda_0 = eigenValues[orderIndices[0]];
            float lambda_1 = eigenValues[orderIndices[1]];
            float lambda_2 = eigenValues[orderIndices[2]];
            Eigen::Vector3f eigVector_0 = eigenVectors[orderIndices[0]];
            Eigen::Vector3f eigVector_1 = eigenVectors[orderIndices[1]];
            Eigen::Vector3f eigVector_2 = eigenVectors[orderIndices[2]];

            // Calculate penalty function.
            Eigen::Vector3f omega = std::max(lambda_1-lambda_0, std::abs(lambda_2-lambda_1-lambda_0))
                    / lambda_2 * eigVector_2;
            // Calculate weight.
            std::vector<std::vector<std::size_t> > edgePair(indices.size());
            std::vector<float> edgeWeights(indices.size());
            for (std::size_t iWeight = 0; iWeight < indices.size(); ++iWeight) {
                edgePair[iWeight] = std::vector<std::size_t>(2);
                // edgePair[iWeight][0] = iPoint;
                edgePair[iWeight][1] = indices[iWeight];
                // edgeWeights[iWeight] =


            }
            // ToDo: The index of the corners need to be stored.
        }

        // ToDo: Detect the edges.

    }

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
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(0.1);
    octree.setInputCloud(inputCloud);
    octree.addPointsFromInputCloud();
    for (std::size_t i = 0; i < corners->size(); ++i) {
        // Extract the neighborhoods of these corners, in which edge detection will be performed.
        pcl::PointXYZ cornerPosition((*corners).points[i].x, (*corners).points[i].y, (*corners).points[i].z);
        std::vector<int> neighborIndices;
        std::vector<float> distances;
        octree.radiusSearch(cornerPosition, 1, neighborIndices, distances);

        pcl::ExtractIndices<pcl::PointXYZ> extractor;
        extractor.setInputCloud(inputCloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr keypointNeiborhoods(new pcl::PointCloud<pcl::PointXYZ>());
        // Need to convert the indices from std::vector<int> to boost::shared_ptr<std::vector<int> >.
        extractor.setIndices(boost::make_shared<std::vector<int> >(neighborIndices));
        extractor.filter(*keypointNeiborhoods);

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
