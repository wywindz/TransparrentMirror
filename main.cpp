#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/cloud_viewer.h>

#include "cvs.h"

int main()
{
    //
    // Specify CVSFeature(s) in the reference model.
    std::vector<RADI::CVSFeature> cvsFeatures(8);
    Eigen::Vector3f xAxis(1.0, 0.0, 0.0);
    Eigen::Vector3f yAxis(0.0, 1.0, 0.0);
    Eigen::Vector3f zAxis(0.0, 0.0, 1.0);
    cvsFeatures[0] = RADI::CVSFeature(pcl::PointXYZ(1.0, 1.0, 1.0));
    cvsFeatures[0].appendVector(-yAxis);
    cvsFeatures[0].appendVector(-zAxis);
    cvsFeatures[0].appendVector(-xAxis);

    cvsFeatures[1] = RADI::CVSFeature(pcl::PointXYZ(1.0, 1.0, -1.0));
    cvsFeatures[1].appendVector(-yAxis);
    cvsFeatures[1].appendVector(-xAxis);
    cvsFeatures[1].appendVector(zAxis);

    cvsFeatures[2] = RADI::CVSFeature(pcl::PointXYZ(1.0, -1.0, 1.0));
    cvsFeatures[2].appendVector(-yAxis);
    cvsFeatures[2].appendVector(xAxis);
    cvsFeatures[2].appendVector(-zAxis);

    cvsFeatures[3] = RADI::CVSFeature(pcl::PointXYZ(1.0, -1.0, -1.0));
    cvsFeatures[3].appendVector(-yAxis);
    cvsFeatures[3].appendVector(xAxis);
    cvsFeatures[3].appendVector(zAxis);

    cvsFeatures[4] = RADI::CVSFeature(pcl::PointXYZ(-1.0, 1.0, 1.0));
    cvsFeatures[4].appendVector(yAxis);
    cvsFeatures[4].appendVector(-xAxis);
    cvsFeatures[4].appendVector(-zAxis);

    cvsFeatures[5] = RADI::CVSFeature(pcl::PointXYZ(-1.0, 1.0, -1.0));
    cvsFeatures[5].appendVector(yAxis);
    cvsFeatures[5].appendVector(-xAxis);
    cvsFeatures[5].appendVector(zAxis);

    cvsFeatures[6] = RADI::CVSFeature(pcl::PointXYZ(-1.0, -1.0, 1.0));
    cvsFeatures[6].appendVector(xAxis);
    cvsFeatures[6].appendVector(yAxis);
    cvsFeatures[6].appendVector(-zAxis);

    cvsFeatures[7] = RADI::CVSFeature(pcl::PointXYZ(-1.0, -1.0, -1.0));
    cvsFeatures[7].appendVector(xAxis);
    cvsFeatures[7].appendVector(yAxis);
    cvsFeatures[7].appendVector(zAxis);

    //
    // Load pcd file.
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneRaw(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("box.pcd", *sceneRaw) == -1) {
        PCL_ERROR("Couldn't read file box.pcd.\n");
        return -1;
    }

    //
    // Filtering & Downsampling.
    // Remove nan data.
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneNaNClean(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*sceneRaw, *sceneNaNClean, indices);
    pcl::io::savePCDFileASCII<pcl::PointXYZ>("box_nan_clean.pcd", *sceneNaNClean);

    // Remove outliers.
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlierFilter;
    outlierFilter.setInputCloud(sceneNaNClean);
    outlierFilter.setMeanK(50);
    outlierFilter.setStddevMulThresh(1.0);
    outlierFilter.filter(*sceneFiltered);

    // Downsampling. Downsample the origin point cloud and reduce the number of points.
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneDownSampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxGrid;
    voxGrid.setInputCloud(sceneFiltered);
    voxGrid.setLeafSize(0.05f, 0.05f, 0.05f);
    voxGrid.filter(*sceneDownSampled);

    std::cout << "Number of points in the pcd file: " << sceneRaw->size() << std::endl;
    std::cout << "Number of points after filtering and downsampling: " << sceneDownSampled->size() << std::endl;

    //
    // Find corners.
    // Note: The output data type of HarrisKeypoint3D should contain intensity.
    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> cornerDetector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr corners(new pcl::PointCloud<pcl::PointXYZI>());
    cornerDetector.setInputCloud(sceneDownSampled);
    cornerDetector.setNonMaxSupression(true);
    cornerDetector.setRadius(0.1);
    cornerDetector.setThreshold(1e-3);
    cornerDetector.compute(*corners);

    std::cout << "Number of corners: " << corners->size() << std::endl;

    // Visualize corners.
    pcl::visualization::PCLVisualizer viewer("Scene");
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(corners, "intensity");
    viewer.addPointCloud(corners, handler, "scene_cloud");
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    //
    // Search edges in the neighborhood of the corners and construct CVSFeatures.

    // // Visualization.
    // pcl::visualization::PCLVisualizer viewer("Scene");
    // viewer.addPointCloud(sceneDownSampled, "scene_cloud");
    // while (!viewer.wasStopped()) {
    //     viewer.spinOnce();
    // }

    // WY::IterativeClosestFace icf;
    // icf.setReferenceModel("./object_test_icf.stl");
    // icf.setScenePointCloud(downSampledCloudScene);
    // std::cout << "Number of triangles in the model mesh: " << icf.getReferenceModel().getNumTriangles() << std::endl;
    // Eigen::Matrix4f initTransf = Eigen::MatrixXf::Identity(4,4);
    // icf.estimate(initTransf);


    /*
    // Compute normals.
    pcl::PointCloud<pcl::Normal>::Ptr modelNormals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr sceneNormals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> normEstimate;
    normEstimate.setKSearch(10);
    normEstimate.setInputCloud(cloudModel);
    normEstimate.compute(*modelNormals);
    normEstimate.setInputCloud(cloudScene);
    normEstimate.compute(*sceneNormals);

    // Downsample clouds to extract keypoints.
    pcl::PointCloud<int>::Ptr modelKeypointIndices(new pcl::PointCloud<int>());
    pcl::PointCloud<int>::Ptr sceneKeypointIndices(new pcl::PointCloud<int>());
    pcl::UniformSampling<pcl::PointXYZRGBA> uniformSampling;
    uniformSampling.setInputCloud(cloudModel);
    uniformSampling.setRadiusSearch(modelSS);
    uniformSampling.compute(*modelKeypointIndices);
    uniformSampling.setInputCloud(cloudScene);
    uniformSampling.setRadiusSearch(sceneSS);
    uniformSampling.compute(*sceneKeypointIndices);

    // Extract keypoints.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr modelKeypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sceneKeypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::copyPointCloud(*cloudModel, modelKeypointIndices->points, *modelKeypoints);
    pcl::copyPointCloud(*cloudScene, sceneKeypointIndices->points, *sceneKeypoints);

    // Compute descriptor for keypoints.
    pcl::PointCloud<pcl::SHOT352>::Ptr modelDescriptors(new pcl::PointCloud<pcl::SHOT352>());
    pcl::PointCloud<pcl::SHOT352>::Ptr sceneDescriptors(new pcl::PointCloud<pcl::SHOT352>());
    pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352> descriptorEstimate;
    descriptorEstimate.setRadiusSearch(descrRad);
    descriptorEstimate.setInputCloud(modelKeypoints);
    descriptorEstimate.setInputNormals(modelNormals);
    descriptorEstimate.setSearchSurface(cloudModel);
    descriptorEstimate.compute(*modelDescriptors);
    descriptorEstimate.setInputCloud(sceneKeypoints);
    descriptorEstimate.setInputNormals(sceneNormals);
    descriptorEstimate.setSearchSurface(cloudScene);
    descriptorEstimate.compute(*sceneDescriptors);

    // Find Model-Scene correspondences with KdTree.
    pcl::CorrespondencesPtr modelSceneCorrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<pcl::SHOT352> matchSearch;
    matchSearch.setInputCloud(modelDescriptors);

    // For each scene keypoint descriptor, find nearest neighbor into the model
    // keypoints descriptor cloud and add it to the correspondences vector.
    for (std::size_t i = 0; i < sceneDescriptors->size(); ++i) {
        std::vector<int> neighIndices(1);
        std::vector<float> neighSqrDists(1);
        if (!pcl_isfinite(sceneDescriptors->at(i).descriptor[0])) { // skipping NaNs.
            continue;
        }
        int foundNeighs = matchSearch.nearestKSearch(sceneDescriptors->at(i), 1, neighIndices, neighSqrDists);
        if (foundNeighs == 1  && neighSqrDists[0] < 0.25f) {
            // add match only if the square descriptor distance is Less than 0.25.
            pcl::Correspondence corr(neighIndices[0], static_cast<int>(i), neighSqrDists[0]);
            modelSceneCorrs->push_back(corr);
        }
    }


    // Output results.
    std::cout << "Model instances found: " << matTransform.size() << std::endl;
    for (std::size_t i = 0; i < matTransform.size(); ++i) {
        std::cout << "\n  Instance " << i + 1 << ":" << std::endl;
        std::cout << "      Correspondences belonging to this instance: "
                  << clusteredCorrs[i].size() << std::endl;

        // Print the transformation matrix.

        printf("\n");
        printf("          | %6.3f %6.3f %6.3f %6.3f | \n", matTransform[i](0,0),
               matTransform[i](0,1), matTransform[i](0,2), matTransform[i](0,3));
        printf("          | %6.3f %6.3f %6.3f %6.3f | \n", matTransform[i](1,0),
               matTransform[i](1,1), matTransform[i](1,2), matTransform[i](1,3));
        printf("          | %6.3f %6.3f %6.3f %6.3f | \n", matTransform[i](2,0),
               matTransform[i](2,1), matTransform[i](2,2), matTransform[i](2,3));
        printf("          | %6.3f %6.3f %6.3f %6.3f | \n", matTransform[i](3,0),
               matTransform[i](3,1), matTransform[i](3,2), matTransform[i](3,3));
    }

    // Visualization.
    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    viewer.addPointCloud(cloudScene, "scene_cloud");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr offSceneModel(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr offSceneModelKeypoints(new pcl::PointCloud<pcl::PointXYZRGBA>());

    if (showCorresponces || showKeypoints) {
        pcl::transformPointCloud(*cloudModel, *offSceneModel, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1,0,0,0));
        pcl::transformPointCloud(*modelKeypoints, *offSceneModelKeypoints, Eigen::Vector3f(-1,0,0), Eigen::Quaternionf(1,0,0,0));

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> offSceneModelColorHandler(offSceneModel, 255, 255, 128);
        viewer.addPointCloud(offSceneModel, offSceneModelColorHandler, "off_scene_model");
    }

    if (showKeypoints) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> sceneKeypointsColorHandler(sceneKeypoints, 0, 0, 255);
        viewer.addPointCloud(sceneKeypoints, sceneKeypointsColorHandler, "scene_keypoints");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scen_keypoints");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> offSceneModelKeypointsColorHandler(sceneKeypoints, 0, 0, 255);
        viewer.addPointCloud(offSceneModelKeypoints, offSceneModelKeypointsColorHandler, "scene_keypoints");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scen_keypoints");
    }

    for (std::size_t i = 0; i < matTransform.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rotatedModel(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::transformPointCloud(*cloudModel, *rotatedModel, matTransform[i]);

        std::stringstream ssCloud;
        ssCloud << "instance" << i;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rotatedModelColorHandler(rotatedModel, 255, 0, 0);
        viewer.addPointCloud(rotatedModel, rotatedModelColorHandler, ssCloud.str());

        if (showCorresponces) {
            for (std::size_t j = 0; j < clusteredCorrs[i].size(); ++j) {
                std::stringstream ssLine;
                ssLine << "correspondence line" << i << "_" << j;
                pcl::PointXYZRGBA & modelPoint = offSceneModelKeypoints->at(clusteredCorrs[i][j].index_query);
                pcl::PointXYZRGBA & scenePoint = sceneKeypoints->at(clusteredCorrs[i][j].index_match);

                viewer.addLine<pcl::PointXYZRGBA, pcl::PointXYZRGBA>(modelPoint, scenePoint, 0, 255, 0, ssLine.str());
            }
        }

    }

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    */

    /*
    //
    // Segmentation.
    //
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloudFiltered);
    seg.segment(*inlierIndices, *coefficients);

    if (inlierIndices->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return -1;
    }

    // Extract the inliers.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extracter;
    extracter.setInputCloud(cloudFiltered);
    extracter.setIndices(inlierIndices);
    extracter.setNegative(false);
    extracter.filter(*cloudPlane);

    */


    std::cout << "Hello World!" << std::endl;
    return 0;
}

