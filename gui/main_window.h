/*
 * GUI -- Main Window.
 */

#ifndef MIRROR_MAIN_WINDOW_H_
#define MIRROR_MAIN_WINDOW_H_

#include <QMainWindow>
#include <QMutex>
#include <QVTKWidget.h>
#include "../kinect2_grabber.h"

namespace radi
{
  class MainWindow : public QMainWindow
  {
    public:
      MainWindow ();
      ~MainWindow ();

    private:
      Kinect2Grabber kinect2_grabber_;
      cv::Mat mat_color_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_show_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_recognition_;
      QMutex * mutex;

      QVTKWidget * viewer_kinect;
      QVTKWidget * viewer_model;

      void
      setWindowParameters ();

      void
      arrangeWidgets ();

      void
      bindSignalSlots ();

  }; // class MainWindow

} // namespace radi

#endif
