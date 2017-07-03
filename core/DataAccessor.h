/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2014/09/11 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: May 19 2014
 *
 */

#ifndef PCLREF_DATA_ACCESSOR_H_
#define PCLREF_DATA_ACCESSOR_H_

#include <PclRefLIB.h>

namespace pclref
{
  /**
  * \brief Abstract class for DataAccessor classes
  */
  class DataAccessor : public boost::enable_shared_from_this<DataAccessor>
  {
  public:
    /// smart pointer type
    typedef boost::shared_ptr<DataAccessor> Ptr;

    virtual ~DataAccessor();

    /// enum to interpret columns as float/double or enum/string
    enum columnType
    {
      FLOAT,
      ENUM
    };

    /// saves accessor as csv-like file
    void toCSV (std::ostream& file, int precision = 3) const;
    /// saves accessor as csv-like file
    void toCSV (std::string& filename, int precision = 3) const;
#ifdef PCLREF_OPENCV
    /// saves accessor as OpenCV matrix
    void toCVMat (cv::Mat& mat) const;
#endif

    /// sets the name of this accessor. Useful for automatic filename generation.
    void setName (const std::string& name);
    /// gets the name of this accessor.
    std::string getName () const;

    /// deactivates enums. Enum typed columns return their ids now.
    virtual void deactivateEnums () = 0;

    /// split accessor into multiple acessors by value of given column.
    void splitByValue(int column, std::vector<typename DataAccessor::Ptr>& views);

    /// gets column count
    virtual size_t getColumnCount () const = 0;
    /// gets row count
    virtual size_t getRowCount () const = 0;
    /// gets column type
    virtual columnType getColumnType (size_t column) const = 0;
    /// gets column name
    virtual std::string getColumnName (size_t column) const = 0;

    /// access a cell
    virtual double at (size_t row, size_t column) const = 0;
    /// access a cell as enum
    virtual std::string atEnum (size_t row, size_t column) const = 0;
    /// sets a cell
    virtual void set(size_t row, size_t column, double value) = 0;
    /// sets a cell
    virtual void set(size_t row, size_t column, const std::string& value) = 0;

    /// interpret columns as pcl::PointXYZ compatible points.
    template<class PointType>
    void interpretAsXYZ(size_t column0, size_t column1, size_t column2,
                        std::vector<PointType>& points)
    {
      size_t rc = getRowCount();
      for(size_t row = 0; row < rc; ++row)
      {
        PointType p;
        p.x = at(row, column0);
        p.y = at(row, column1);
        p.z = at(row, column2);
        points.push_back(p);
      }
    }

    /** \brief Interpret an area of the table as a transformation represented as an Eigen::Matrix4f
      * \param column0 starting column
      * \param row0 starting row
      * \param mat resulting transformation
      * \param inlined matrix interpretation mode
      * \param from_matrix interpret input as matrix
      *
      * This method creates a Transformation starting at (row0, column0). If from_matrix is true
      * in inlined mode it loads a matrix as a line to (row0, column0 + 15). In rectangle mode it loads the
      * transformation as a rectangle to (row0 + 3, column0 + 3).
      * If from_matrix is false the input is handled as a euler angles + translation.
      */
    void interpretAsTransformation(size_t column0, size_t row0, Eigen::Matrix4f& mat, bool inlined = true, bool from_matrix = true);

    /** \brief Interpret an area of the table as Eigen::Matrix
      * \param column0 starting column
      * \param row0 starting row
      * \param mat resulting transformation
      * \param inlined matrix interpretation mode
      *
      * This method creates a Matrix starting at (row0, column0). In inlined mode
      * it loads a matrix as a line to (row0, column0 + 15). In rectangle mode it loads the
      * transformation as a rectangle to (row0 + 3, column0 + 3)
      */
    template<typename Real>
    void interpretAsEigenMatrix(size_t column0, size_t row0,
                                Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic>& mat, bool inlined = true)
    {
      for(size_t j = 0; j < mat.rows(); ++j)
      {
        for(size_t i = 0; i < mat.cols(); ++i)
        {
          mat(j,i) = inlined ?
                     at(row0,     column0 + i + mat.cols()*j) :
                     at(row0 + j, column0 + i);
        }
      }
    }
    /// write an Eigen::Matrix starting at (column0, row0).
    template<typename Real>
    void writeEigenMatrix(size_t column0, size_t row0,
                          const Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic>& mat, bool inlined = true)
    {
      for(size_t j = 0; j < mat.rows(); ++j)
      {
        for(size_t i = 0; i < mat.cols(); ++i)
        {
          inlined ?
          set(row0,     column0 + i + mat.cols()*j, mat(j,i)) :
          set(row0 + j, column0 + i, mat(j,i));
        }
      }
    }
    /// Interpret an area of the table as Eigen::Matrix4f starting at (column0, row0).
    void interpretAsEigenMatrix(size_t column0, size_t row0,
                                Eigen::Matrix4f& mat, bool inlined = true)
    {
      for(size_t j = 0; j < mat.rows(); ++j)
      {
        for(size_t i = 0; i < mat.cols(); ++i)
        {
          mat(j,i) = inlined ?
                     at(row0,     column0 + i + mat.cols()*j) :
                     at(row0 + j, column0 + i);
        }
      }
    }
    /// write an Eigen::Matrix4f starting at (column0, row0).
    void writeEigenMatrix(size_t column0, size_t row0,
                          const Eigen::Matrix4f& mat, bool inlined = true)
    {
      for(size_t j = 0; j < 4; ++j)
      {
        for(size_t i = 0; i < 4; ++i)
        {
          inlined ?
          set(row0,     column0 + i + mat.cols()*j, mat(j,i)) :
          set(row0 + j, column0 + i, mat(j,i));
        }
      }
    }
    /// Interpret an area of the table as vector starting at (column0, row0), always line-wise.
    template<typename Real>
    void interpretAsEigenVector(size_t column0, size_t row0,
                                Eigen::Matrix<Real, Eigen::Dynamic,1>& vec)
    {
      for(size_t i = 0; i < vec.rows(); ++i)
      {
        vec(i) = at(row0, column0 + i);
      }
    }
    /// Write a vector starting at (column0, row0), always line-wise.
    template<typename Real>
    void writeEigenVector(size_t column0, size_t row0,
                          const Eigen::Matrix<Real, Eigen::Dynamic,1>& vec)
    {
      for(size_t i = 0; i < vec.rows(); ++i)
      {
        set(row0, column0 + i, (double)vec(i));
      }
    }

    /// internal usage, use at own risk
    virtual size_t _getEnumSize (size_t column) const = 0;
    /// internal usage, use at own risk
    virtual double _enumToValue (const std::string& str, size_t column) const = 0;
    /// internal usage, use at own risk
    virtual std::string _valueToEnum (double value, size_t column) const = 0;

  private:
    std::string m_name;

  };

}

#endif /* DATA_ACCESSOR_H_ */
