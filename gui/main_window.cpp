#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);

#include <QHBoxLayout>
#include <QWidget>

#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>

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
    setWindowTitle ("Transparent Mirror");
  }

  void
  MainWindow::arrangeWidgets ()
  {
    viewer_kinect = new QVTKWidget ();
    viewer_model = new QVTKWidget ();

    // // Sphere
    // vtkSmartPointer<vtkSphereSource> sphereSource =
    //   vtkSmartPointer<vtkSphereSource>::New();
    // sphereSource->Update();
    // vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
    //   vtkSmartPointer<vtkPolyDataMapper>::New();
    // sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
    // vtkSmartPointer<vtkActor> sphereActor =
    //   vtkSmartPointer<vtkActor>::New();
    // sphereActor->SetMapper(sphereMapper);

    // // VTK Renderer
    // vtkSmartPointer<vtkRenderer> renderer =
    //   vtkSmartPointer<vtkRenderer>::New();
    // renderer->AddActor(sphereActor);
    // viewer_kinect->GetRenderWindow()->AddRenderer(renderer);

    setCentralWidget(viewer_kinect);
    //QWidget * widget = new QWidget (this);

    //QHBoxLayout * layout_main = new QHBoxLayout ();
    //layout_main->addWidget(viewer_kinect);
    // // layout_main->addWidget (viewer_kinect);
    // // layout_main->addWidget (viewer_model);

    // widget->setLayout(layout_main);
  }

  void
  MainWindow::bindSignalSlots ()
  {

  }

} // namespace radi
