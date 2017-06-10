/*
 * GUI -- Thread for recognizing the pose of the objects.
 */

#ifndef MIRROR_THREAD_RECOGNITION_H_
#define MIRROR_THREAD_RECOGNITION_H_

#include <QThread>
#include <QMutex>
#include "../kinect2_grabber.h"

namespace radi
{
  class ThreadRecognition : public QThread
  {
    public:
      ThreadRecognition (QMutex * mutex, Kinect2Grabber * kinect2_grabber);
      ~ThreadRecognition ();

      void
      run ();

    private:
      QMutex * mutex_;
      Kinect2Grabber * kinect2_grabber_;

  }; // class MainWindow

} // namespace radi

#endif
