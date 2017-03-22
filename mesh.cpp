#include <cmath>
#include <qt4/QtCore/QFile>
#include <qt4/QtCore/QString>
#include <qt4/QtCore/QStringList>
#include <qt4/QtCore/QTextStream>
#include "mesh.h"

namespace radi
{
  Mesh::Mesh () : triangles_(std::vector<std::vector<Eigen::Vector3f> >())
  { }

  Mesh::~Mesh ()
  { }

  // Load '.stl' file.
  void
  Mesh::loadModel (const std::string & filePath)
  {
    QFile fileModel (QString::fromStdString(filePath));
    if (!fileModel.open (QIODevice::ReadOnly | QIODevice::Text))
    {
      std::cerr << "Cannot open file: " + filePath + "." << std::endl;
    }

    QTextStream inStream (&fileModel);
    while (!inStream.atEnd ())
    {
      QString line = inStream.readLine ().trimmed ();
      if (line.startsWith ("outer loop"))
      {
        std::vector<Eigen::Vector3f> triangle;
        while (!line.startsWith ("endloop"))
        {
          line = inStream.readLine ().trimmed ();
          if (line.startsWith ("vertex"))
          {
            QStringList vertexData = line.split (" ");
            Eigen::Vector3f vertex;
            vertex[0] = vertexData[1].toFloat ();
            vertex[1] = vertexData[2].toFloat ();
            vertex[2] = vertexData[3].toFloat ();
            triangle.push_back (vertex);
          }
        }
        triangles_.push_back (triangle);
      }
    }
  }

  const std::vector<Eigen::Vector3f> &
  Mesh::getTriangle (std::size_t index) const
  {
      return triangles_[index];
  }

} // namespace radi
