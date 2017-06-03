/*
 * Kinect2Grabber based on
 * https://github.com/giacomodabisias/libfreenect2pclgrabber
*/

#ifndef MIRROR_KINECT2_GRABBER_H_
#define MIRROR_KINECT2_GRABBER_H_

#define WITH_PCL

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>
#include <signal.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <chrono>
#include <Eigen/Core>

namespace radi {

  typedef libfreenect2::Freenect2Device::IrCameraParams IrCameraParams;
  typedef libfreenect2::Freenect2Device::ColorCameraParams ColorCameraParams;

  enum ProcessorType { CPU, OPENCL, OPENGL, CUDA };

  class Kinect2Grabber
  {
    public:
      Kinect2Grabber (ProcessorType processor_type = OPENGL, bool mirror = false, std::string serial_number = std::string());
      ~Kinect2Grabber();

      IrCameraParams
      getIrParameters ();

      ColorCameraParams
      getRgbParameters ();

      void
      disableLog ();

      void
      enableLog ();

      void printParameters ();

      void storeParameters ();

#ifdef WITH_PCL
      void
      getPointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

    private:

      void
      updatePointCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);
#endif

    public:
      void
      shutDown ();

      inline void
      mirror () { mirror_ != mirror_; }

      inline libfreenect2::SyncMultiFrameListener *
      getListener ()
      {
        return (&listener_);
      }

      // Use only if you want only depth, else use get(cv::Mat, cv::Mat) to have the images aligned
      void
      getDepth (cv::Mat depth_mat);

      void
      getIr (cv::Mat ir_mat);

      // Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
      void
      getColor (cv::Mat & color_mat);

      // Depth and color are aligned and registered
      void
      get (cv::Mat & color_mat, cv::Mat & depth_mat, const bool full_hd = true, const bool remove_points = false);

      // Depth and color are aligned and registered
      void
      get (cv::Mat & color_mat, cv::Mat & depth_mat, cv::Mat & ir_mat, const bool full_hd = true, const bool remove_points = false);

#ifdef WITH_PCL
      // All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
      void
      get (cv::Mat & color_mat, cv::Mat & depth_mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
          const bool full_hd = true, const bool remove_points = false);
#endif

#ifdef WITH_SERIALIZATION
      inline void
      enableSerialization ()
      {
        serialize_ = true;
      }

      inline void
      disableSerialization ()
      {
        serialize_ = false;
      }

      inline bool
      getSerializeStatus ()
      {
        return (serialize_);
      }
#endif

    private:
      libfreenect2::Freenect2 freenect2_;
      libfreenect2::Freenect2Device * device_ = nullptr;
      libfreenect2::PacketPipeline * pipeline_ = nullptr;
      libfreenect2::Registration * registration_ = nullptr;
      libfreenect2::SyncMultiFrameListener listener_;
      libfreenect2::Logger * logger_ = nullptr;
      libfreenect2::FrameMap frames_;
      libfreenect2::Frame undistorted_, registered_, big_mat_;
      Eigen::Matrix<float,512,1> colmap;
      Eigen::Matrix<float,424,1> rowmap;
      std::string serial_number_;
      int map_[512 * 424];
      float qnan_;
      bool mirror_;

#ifdef WITH_SERIALIZATION
      bool serialize_;
      std::ofstream * file_streamer_;
      boost::archive::binary_oarchive * oa_;
#endif

#ifdef WITH_SERIALIZATION
      void
      serializeFrames (const cv::Mat & depth, const cv::Mat & color);

  #ifdef WITH_PCL
      void serializeCloud (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  #endif

#endif

      void
      prepareMake3D (const IrCameraParams & depth_p);
  };

}

#endif
