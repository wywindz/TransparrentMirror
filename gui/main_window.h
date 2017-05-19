/*
 * GUI -- Main Window.
 */

#ifndef MIRROR_MAIN_WINDOW_H_
#define MIRROR_MAIN_WINDOW_H_

// #include <QMainWindow>
// #include <QVTKWidget.h>

namespace radi
{
  class MainWindow //: public QMainWindow
  {
    // Q_OBJECT

    public:
      MainWindow ();
      ~MainWindow ();

    private:
      // QVTKWidget * viewer_kinect;
      // QVTKWidget * viewer_model;

      void
      setWindowParameters ();

      void
      arrangeWidgets ();

      void
      bindSignalSlots ();

  }; // class MainWindow

} // namespace radi

#endif
