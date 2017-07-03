/*
 * data_view.cc
 *
 *  Created on: May 19, 2014
 *      Author: max
 */

#include <DataView.h>
using namespace pclref;

DataView::DataView ()
{

}

void DataView::setAccessor (boost::shared_ptr<DataAccessor> accessor)
{
  m_accessor = accessor;
  m_columns.clear ();
  m_rows.clear ();
}
void DataView::addColumn (size_t column)
{
  m_columns.push_back (column);
}
void DataView::addColumns (const std::vector<size_t>& columns)
{
  m_columns.insert (m_columns.end(), columns.begin(), columns.end());
}
void DataView::addRow (size_t row)
{
  m_rows.push_back (row);
}
void DataView::addRows (const std::vector<size_t>& rows)
{
  m_rows.insert (m_rows.end (), rows.begin (), rows.end ());
}

size_t DataView::_getEnumSize (size_t column) const
{
  return m_accessor->_getEnumSize (m_columns[column]);
}
double DataView::_enumToValue (const std::string& str, size_t column) const
{
  return m_accessor->_enumToValue (str, m_columns[column]);
}
std::string DataView::_valueToEnum (double value, size_t column) const
{
  return m_accessor->_valueToEnum (value, m_columns[column]);
}

void DataView::deactivateEnums ()
{
  m_accessor->deactivateEnums ();
}

size_t DataView::getColumnCount () const
{
  return m_columns.size ();
}
size_t DataView::getRowCount () const
{
  return m_rows.size();
}
double DataView::at (size_t row, size_t column) const
{
  return m_accessor->at (m_rows[row], m_columns[column]);
}
std::string DataView::atEnum (size_t row, size_t column) const
{
  return m_accessor->atEnum (m_rows[row], m_columns[column]);
}
void DataView::set(size_t row, size_t column, double value)
{
  m_accessor->set(m_rows[row], m_columns[column], value);
}
void DataView::set(size_t row, size_t column, const std::string& value)
{
  m_accessor->set(m_rows[row], m_columns[column], value);
}

DataAccessor::columnType DataView::getColumnType (size_t column) const
{
  return m_accessor->getColumnType (m_columns[column]);
}
std::string DataView::getColumnName (size_t column) const
{
  return m_accessor->getColumnName (m_columns[column]);
}
