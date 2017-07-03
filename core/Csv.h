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

#ifndef PCLREF_CSV_H_
#define PCLREF_CSV_H_

#include <PclRefLIB.h>

namespace pclref
{
  /**
  * \class CSVRow
  *
  * \brief A row of a CSV like file
  */
  class CSVRow
  {
  public:
    /// constructor
    CSVRow(char seperator);

    /// subscript operator
    const std::string& operator[] (size_t index) const;
    /// stl-style size
    size_t size () const;

    /// reads in a row of data
    void read (std::istream& str);

  private:
    char m_seperator;
    std::vector<std::string> m_data;
  };

  std::istream& operator>>(std::istream& str, CSVRow& data);

  /**
  * \class CSVIterator
  *
  * \brief Wrapper for easy CSV reading
  */
  class CSVIterator
  {
  public:
    typedef std::input_iterator_tag iterator_category;
    typedef CSVRow value_type;
    typedef size_t difference_type;
    typedef CSVRow* pointer;
    typedef CSVRow& reference;

    /// constructor
    CSVIterator (std::istream& str, char seperator);
    /// default constructor
    CSVIterator ();

    /// pre increment
    CSVIterator& operator++ ();
    /// post increment
    CSVIterator operator++(int);
    /// dereference
    const CSVRow& operator*() const;
    /// structure dereference
    const CSVRow* operator->() const;

    /// comparison
    bool operator==(const CSVIterator& rhs);
    /// comparison
    bool operator!=(const CSVIterator& rhs);

    /// streaming
    friend std::istream& operator>>(std::istream& str, CSVRow& data);

  private:
    std::istream* m_str;
    CSVRow m_row;
  };

}

#endif
