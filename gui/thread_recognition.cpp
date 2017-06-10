#include "thread_recognition.h"

namespace radi
{
  ThreadRecognition::ThreadRecognition (QMutex * mutex, Kinect2Grabber * kinect2_grabber)
      : mutex_ (mutex), kinect2_grabber_ (kinect2_grabber)
  { }

  ThreadRecognition::~ThreadRecognition ()
  { }

  void
  ThreadRecognition::run ()
  {
  }

} // namespace radi
