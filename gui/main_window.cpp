// #include <QHBoxLayout>

#include "main_window.h"

namespace radi
{
  MainWindow::MainWindow ()
  {
    setWindowParameters ();
    arrangeWidgets ();
    bindSignalSlots ();
  }

  MainWindow::~MainWindow ()
  { }

  void
  MainWindow::setWindowParameters ()
  {
    // setGeometry (50, 50, 800, 600);
    // setWindowTitle ("Transparent Mirror");
  }

  void
  MainWindow::arrangeWidgets ()
  {
    // viewer_kinect = new QVTKWidget (this);
    // viewer_model = new QVTKWidget (this);

    // QHBoxLayout * layout_main = new QHBoxLayout (this);
    // layout_main->addWidget (viewer_kinect);
    // layout_main->addWidget (viewer_model);
  }

  void
  MainWindow::bindSignalSlots ()
  {

  }

} // namespace radi
