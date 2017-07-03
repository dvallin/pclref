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

#ifndef PCLREF_DATA_VIEW_H_
#define PCLREF_DATA_VIEW_H_

#include <PclRefLIB.h>
#include <DataAccessor.h>

namespace pclref
{
  /**
  * \brief this class can be used to create subsets of a DataAccessor
  * By adding an accessor, columns and rows to be viewed any possible
  * subset of a DataAccessor can be created. Even duplicate rows or columns
  * can be used.
  */
  class DataView : public DataAccessor
  {
  public:
    typedef boost::shared_ptr<DataView> Ptr;

    /// constructor
    DataView ();

    /// input accessor
    void setAccessor (boost::shared_ptr<DataAccessor> accessor);
    /// adds a column to the accessor
    void addColumn (size_t column);
    /// adds columns to the accessor
    void addColumns (const std::vector<size_t>& columns);
    /// adds row to the accessor
    void addRow (size_t row);
    /// adds rows to the accessor
    void addRows (const std::vector<size_t>& rows);

    virtual void deactivateEnums ();

    // getters
    virtual size_t getColumnCount () const;
    virtual size_t getRowCount () const;
    virtual columnType getColumnType (size_t column) const;
    virtual std::string getColumnName (size_t column) const;

    virtual double at (size_t row, size_t column) const;
    virtual std::string atEnum (size_t row, size_t column) const;
    virtual void set (size_t row, size_t column, double value);
    virtual void set (size_t row, size_t column, const std::string& value);

    //internal:
    virtual size_t _getEnumSize (size_t column) const;
    virtual double _enumToValue (const std::string& str, size_t column) const;
    virtual std::string _valueToEnum (double value, size_t column) const;

  private:
    boost::shared_ptr<DataAccessor> m_accessor;
    std::vector<size_t> m_columns;
    std::vector<size_t> m_rows;
  };

}


#endif /* DATA_VIEW_H_ */
