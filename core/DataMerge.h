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

#ifndef PCLREF_DATA_MERGE_H_
#define PCLREF_DATA_MERGE_H_

#include <PclRefLIB.h>
#include <DataAccessor.h>

namespace pclref
{
  /**
  * \brief Represents multiple DataAccessors as one.
  * see DataAccessor for documentation of interface.
  */
  class DataMerge : public DataAccessor
  {
  public:
    typedef boost::shared_ptr<DataMerge> Ptr;
    typedef std::pair<size_t, boost::shared_ptr<DataAccessor> > accessor_offset_pair;
    typedef std::vector<accessor_offset_pair> accessor_offset_array;

    /// constructor
    DataMerge (bool rowMerge = true);
    virtual ~DataMerge();

    /** \brief Add a new Accessor to the DataMerge
      * \param accessor a pointer to a DataAccessor object to be merged
      * This accessor will be merged to the end or the right (depending on rowMerge).
      */
    void addAccessor (boost::shared_ptr<DataAccessor> accessor);

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

    //internal:
    virtual size_t _getEnumSize (size_t column) const;
    virtual double _enumToValue (const std::string& str, size_t column) const;
    virtual std::string _valueToEnum (double value, size_t column) const;

  private:
    DISALLOW_COPY_AND_ASSIGN (DataMerge);

    accessor_offset_array m_accessors;
    bool m_row_merge;
  };

}

#endif
