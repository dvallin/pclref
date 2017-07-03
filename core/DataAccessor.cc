/*
 * data_accessor.cc
 *
 *  Created on: May 19, 2014
 *      Author: max
 */

#include <DataAccessor.h>
#include <DataView.h>
#include <Arrays.h>
using namespace pclref;

DataAccessor::~DataAccessor()
{
}

void DataAccessor::toCSV (std::string& filename, int precision) const
{
  std::ofstream file(filename.c_str());
  toCSV(file, precision);
  file.close();
}
void DataAccessor::toCSV (std::ostream& file, int precision) const
{
  const size_t columns = getColumnCount ();
  const size_t rows = getRowCount ();
  if(columns == 0 || rows == 0)
    return;
  for (size_t i = 0; i < columns; ++i)
  {
    file << getColumnName(i);

    file << ":";

    switch (getColumnType(i))
    {
    case FLOAT:
      file << "f";
      break;
    case ENUM:
      file << "e";
      break;
    }
    if (i < columns - 1)
      file << ',';
  }
  file << '\n';

  for (size_t j = 0; j < rows; ++j)
  {
    std::string value;
    for (size_t i = 0; i < columns; ++i)
    {
      switch (getColumnType (i))
      {
      //case FLOAT: value = to_string (at (j, i)); break; c++11
      case FLOAT:
      {
        value = precision_cast(at(j,i), precision);
      }
      break;
      case ENUM:
        value = atEnum (j, i);
        break;
      }

      file << value;
      if (i < columns - 1)
        file << ',';
    }
    file << '\n';
  }
}
#ifdef PCLREF_OPENCV
void DataAccessor::toCVMat (cv::Mat& mat) const
{
  const size_t columns = getColumnCount();
  const size_t rows = getRowCount ();
  mat.create (rows, columns, CV_32FC1);
  for (size_t j = 0; j < rows; ++j)
  {
    for (size_t i = 0; i < columns; ++i)
    {
      mat.at<float> (j, i) = static_cast<float>(at (j, i));
    }
  }
}
#endif

void DataAccessor::splitByValue(int column, std::vector<typename DataAccessor::Ptr>& views)
{
  typedef std::pair<double, std::vector<size_t> > ValueIndicesPair;

  size_t rc = getRowCount();
  std::map<double, std::vector<size_t> > rindices;
  std::vector<size_t> cindices;
  for(size_t row = 0; row < rc; ++row)
  {
    rindices[at(row, column)].push_back(row);
  }
  Arrays::natural(0, getColumnCount(), cindices);

  BOOST_FOREACH(ValueIndicesPair pair, rindices)
  {
    typename DataView::Ptr view = typename DataView::Ptr(new DataView());
    view->setAccessor(shared_from_this());
    view->addColumns(cindices);
    view->addRows(pair.second);
    views.push_back(view);
  }
}
void DataAccessor::interpretAsTransformation(size_t column0, size_t row0, Eigen::Matrix4f& mat, bool inlined, bool from_matrix)
{
  if(from_matrix)
  {
    for(size_t j = 0; j < 4; ++j)
    {
      for(size_t i = 0; i < 4; ++i)
      {
        mat(j,i) = inlined ?
                   at(row0,     column0 + i + 4*j) :
                   at(row0 + j, column0 + i);
      }
    }
  }
  else
  {
    Eigen::Translation3f translation;
    translation.x() = at(row0, column0 + 0);
    translation.y() = at(row0, column0 + 1);
    translation.z() = at(row0, column0 + 2);

    Eigen::AngleAxisf rollAngle(at(row0, column0 + 3), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(at(row0, column0 + 4), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(at(row0, column0 + 5), Eigen::Vector3f::UnitZ());
    Eigen::Transform<float, 3, Eigen::Affine> transform = translation * yawAngle * pitchAngle * rollAngle;
    mat = transform.matrix();
  }
}
void DataAccessor::setName(const std::string& name)
{
  m_name = name;
}

std::string DataAccessor::getName() const
{
  return m_name;
}

