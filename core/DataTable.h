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

#ifndef PCLREF_DATA_TABLE_H_
#define PCLREF_DATA_TABLE_H_

#include <PclRefLIB.h>
#include <DataAccessor.h>

namespace pclref
{
  /**
  * \brief a table of data
  */
  class DataTable : public DataAccessor
  {
  public:
    typedef boost::shared_ptr<DataTable> Ptr;
    typedef std::map<std::string, double> enum_to_value;
    typedef std::map<double, std::string> value_to_enum;

    /// default constructor
    DataTable ();
    /// constructor
    DataTable (const std::vector<DataAccessor::columnType>& columnTypes,
               const std::vector<std::string>& columnNames);
    /// destructor
    virtual ~DataTable ();


    /** \brief Initializes the columns meta data of this DataTable
      * \param columnTypes a vector of columnTypes
      * \param columnNames a vector of columnNames
      * this init function must be called before any further usage.
      */
    void init(const std::vector<DataAccessor::columnType>& columnTypes,
              const std::vector<std::string>& columnNames);

    /// load a table from a csv-like file
    void fromCSV (const std::string& filename, bool has_header, char seperator = ',');
    /// load a table from a csv-like file
    void fromCSV (std::ifstream& file, bool has_header, char seperator = ',');
    /// load a table from an array
    void fromData (const std::vector<double> data, size_t rows, size_t cols);
#ifdef PCLREF_OPENCV
    /// load a table from an OpenCV matrix
    void fromCvMat (cv::Mat& mat);
#endif

    virtual void deactivateEnums ();

    // getters
    virtual size_t getColumnCount () const;
    virtual size_t getRowCount () const;
    virtual columnType getColumnType (size_t column) const;
    virtual std::string getColumnName (size_t column) const;

    virtual double at (size_t row, size_t column) const;
    virtual std::string atEnum (size_t row, size_t column) const;
    virtual void set(size_t row, size_t column, double value);
    virtual void set(size_t row, size_t column, const std::string& value);


    //internal usage, use at own risk!
    virtual size_t _getEnumSize (size_t column) const;
    virtual double _enumToValue (const std::string& str, size_t column) const;
    virtual std::string _valueToEnum (double value, size_t column) const;
    void _ensure(size_t row);
    void _expandBy(size_t rows, bool locking = true);

  private:
    DISALLOW_COPY_AND_ASSIGN (DataTable);

    std::vector<DataAccessor::columnType> m_column_types;
    std::vector<std::string> m_column_names;
    std::vector<double> m_data;

    std::vector<identifier> m_enum_ids;
    std::vector<enum_to_value> m_enum_to_value_converters;
    std::vector<value_to_enum> m_value_to_enum_converters;

    size_t m_column_count;
    size_t m_row_count;

#ifdef PCLREF_PARALLEL
    omp_lock_t m_writelock;
#endif
  };

}


#endif /* DATA_TABLE_H_ */
