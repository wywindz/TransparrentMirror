#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/cloud_viewer.h>

#include "cvs.h"
#include "cvs_estimation.h"

int main()
{

    //
    // Specify CVSFeature(s) in the reference model.
    std::vector<radi::CVSFeature> cvsFeatures(8);
    Eigen::Vector3f xAxis(1.0, 0.0, 0.0);
    Eigen::Vector3f yAxis(0.0, 1.0, 0.0);
    Eigen::Vector3f zAxis(0.0, 0.0, 1.0);
    cvsFeatures[0] = radi::CVSFeature(pcl::PointXYZ(1.0, 1.0, 1.0));
    cvsFeatures[0].appendVector(-yAxis);
    cvsFeatures[0].appendVector(-zAxis);
    cvsFeatures[0].appendVector(-xAxis);

    cvsFeatures[1] = radi::CVSFeature(pcl::PointXYZ(1.0, 1.0, -1.0));
    cvsFeatures[1].appendVector(-yAxis);
    cvsFeatures[1].appendVector(-xAxis);
    cvsFeatures[1].appendVector(zAxis);

    cvsFeatures[2] = radi::CVSFeature(pcl::PointXYZ(1.0, -1.0, 1.0));
    cvsFeatures[2].appendVector(-yAxis);
    cvsFeatures[2].appendVector(xAxis);
    cvsFeatures[2].appendVector(-zAxis);

    cvsFeatures[3] = radi::CVSFeature(pcl::PointXYZ(1.0, -1.0, -1.0));
    cvsFeatures[3].appendVector(-yAxis);
    cvsFeatures[3].appendVector(xAxis);
    cvsFeatures[3].appendVector(zAxis);

    cvsFeatures[4] = radi::CVSFeature(pcl::PointXYZ(-1.0, 1.0, 1.0));
    cvsFeatures[4].appendVector(yAxis);
    cvsFeatures[4].appendVector(-xAxis);
    cvsFeatures[4].appendVector(-zAxis);

    cvsFeatures[5] = radi::CVSFeature(pcl::PointXYZ(-1.0, 1.0, -1.0));
    cvsFeatures[5].appendVector(yAxis);
    cvsFeatures[5].appendVector(-xAxis);
    cvsFeatures[5].appendVector(zAxis);

    cvsFeatures[6] = radi::CVSFeature(pcl::PointXYZ(-1.0, -1.0, 1.0));
    cvsFeatures[6].appendVector(xAxis);
    cvsFeatures[6].appendVector(yAxis);
    cvsFeatures[6].appendVector(-zAxis);

    cvsFeatures[7] = radi::CVSFeature(pcl::PointXYZ(-1.0, -1.0, -1.0));
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
    voxGrid.setLeafSize(0.01f, 0.01f, 0.01f);
    voxGrid.filter(*sceneDownSampled);

    std::cout << "Number of points in the pcd file: " << sceneRaw->size() << std::endl;
    std::cout << "Number of points after filtering and downsampling: " << sceneDownSampled->size() << std::endl;

    radi::CVSEstimation cvsEstimation;
    cvsEstimation.setInputCloud(sceneDownSampled);
    cvsEstimation.esimate();

    // // Visualize corners.
    // pcl::visualization::PCLVisualizer viewer("Scene");
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(corners, "intensity");
    // viewer.addPointCloud(sceneDownSampled, "Scene Downsmapled");
    // viewer.addPointCloud(corners, handler, "Corners");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "Corners");
    // while (!viewer.wasStopped()) {
    //     viewer.spinOnce();
    // }


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

