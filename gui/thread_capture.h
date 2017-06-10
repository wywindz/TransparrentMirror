/*
 * GUI -- Thread for capturing the picture and point cloud which are only used to show the scene.
 */

#ifndef MIRROR_THREAD_CAPTURE_H_
#define MIRROR_THREAD_CAPTURE_H_

#include <QThread>
#include <QMutex>
#include "../kinect2_grabber.h"

namespace radi
{
  class ThreadCapture : public QThread
  {
    public:
      ThreadCapture (QMutex * mutex, Kinect2Grabber * kinect2_grabber);
      ~ThreadCapture ();

      void
      run ();

    private:
      QMutex * mutex;
      Kinect2Grabber * kinect2_grabber_;

  }; // class MainWindow

} // namespace radi

#endif
