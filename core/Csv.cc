/*
 * csv.cc
 *
 *  Created on: May 19, 2014
 *      Author: max
 */

#include <Csv.h>
using namespace pclref;

// CsvRow Implementations
CSVRow::CSVRow(char seperator)
{
  m_seperator = seperator;
}

const std::string& CSVRow::operator[] (size_t index) const
{
  return m_data[index];
}
size_t CSVRow::size () const
{
  return m_data.size ();
}
void CSVRow::read (std::istream& str)
{
  std::string line;
  std::string cell;

  std::getline (str, line);
  std::stringstream lineStream (line);

  m_data.clear ();
  while (std::getline (lineStream, cell, m_seperator))
  {
    m_data.push_back (cell);
  }
}

// CSVIterator Implementations
CSVIterator::CSVIterator (std::istream& str, char seperator)
  : m_str(str.good() ? &str : NULL_PTR)
  , m_row(seperator)
{
  ++(*this);
}
CSVIterator::CSVIterator () : m_str(NULL_PTR), m_row(',')
{
}
CSVIterator& CSVIterator::operator++ ()
{
  if (m_str)
  {
    (*m_str) >> m_row;
    m_str = m_str->good () ? m_str : NULL_PTR;
  }
  return *this;
}
CSVIterator CSVIterator::operator++(int)
{
  CSVIterator tmp (*this);
  ++(*this);
  return tmp;
}
const CSVRow& CSVIterator::operator*() const
{
  return m_row;
}
const CSVRow* CSVIterator::operator->() const
{
  return &m_row;
}
bool CSVIterator::operator==(const CSVIterator& rhs)
{
  return ((this == &rhs) || ((this->m_str == NULL_PTR) && (rhs.m_str == NULL_PTR)));
}
bool CSVIterator::operator!=(const CSVIterator& rhs)
{
  return !((*this) == rhs);
}

// Operators Implemenations
namespace pclref
{
  std::istream& operator>>(std::istream& str, CSVRow& data)
  {
    data.read (str);
    return str;
  }
}
