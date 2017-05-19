#include <cmath>
#include <QFile>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QTextStream>
#include "mesh.h"

namespace radi
{
  Mesh::Mesh () : triangles_(std::vector<std::vector<Eigen::Vector3f> >())
  { }

  Mesh::~Mesh ()
  { }

  // Load '.stl' file.
  void
  Mesh::loadModel (const std::string & file_path)
  {
    QFile file_model (QString::fromStdString(file_path));
    if (!file_model.open (QIODevice::ReadOnly | QIODevice::Text))
    {
      std::cerr << "Cannot open file: " + file_path + "." << std::endl;
    }

    QTextStream inStream (&file_model);
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
      return (triangles_[index]);
  }

} // namespace radi
