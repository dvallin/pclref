/*
 * data_merge.cc
 *
 *  Created on: May 19, 2014
 *      Author: max
 */

#include <DataMerge.h>
using namespace pclref;

bool accessor_offset_pair_comp (const DataMerge::accessor_offset_pair& i,
                                const DataMerge::accessor_offset_pair& j)
{
  return (i.first < j.first);
}

DataMerge::DataMerge (bool rowMerge) : m_row_merge (rowMerge)
{
}
DataMerge::~DataMerge()
{
  m_accessors.clear();
}

void DataMerge::addAccessor (boost::shared_ptr<DataAccessor> accessor)
{
  size_t c = m_accessors.size ();
  if (IS_VALID_ID(c))
  {
    accessor_offset_pair& last = m_accessors[c - 1];
    size_t offset = (m_row_merge ? last.second->getRowCount () : last.second->getColumnCount ());
    if (c > 1)
    {
      accessor_offset_pair prev = m_accessors[c - 2];
      offset += prev.first;
    }
    last.first = offset;
  }
  m_accessors.push_back (accessor_offset_pair(std::numeric_limits<size_t>::max(), accessor));
}

size_t DataMerge::_getEnumSize (size_t column) const
{
  size_t value;
  if (!m_row_merge)
  {
    boost::shared_ptr<DataAccessor> a;
    size_t c;
    accessor_offset_pair comp (column, boost::shared_ptr<DataAccessor>());
    accessor_offset_array::const_iterator bound =
      std::upper_bound (m_accessors.begin (), m_accessors.end (), comp, accessor_offset_pair_comp);

    a = bound->second;

    if (bound != m_accessors.begin ())
    {
      --bound;
      c = column - bound->first;
    }
    else
    {
      c = column;
    }
    value = a->_getEnumSize (c);
  }
  else
  {
    value = m_accessors.front ().second->_getEnumSize (column);
  }
  return value;
}
double DataMerge::_enumToValue (const std::string& str, size_t column) const
{
  double value;
  if (!m_row_merge)
  {
    boost::shared_ptr<DataAccessor> a;
    size_t c;
    accessor_offset_pair comp (column, boost::shared_ptr<DataAccessor>());
    accessor_offset_array::const_iterator bound =
      std::upper_bound (m_accessors.begin (), m_accessors.end (), comp, accessor_offset_pair_comp);

    a = bound->second;

    if (bound != m_accessors.begin ())
    {
      --bound;
      c = column - bound->first;
    }
    else
    {
      c = column;
    }
    value = a->_enumToValue (str, c);
  }
  else
  {
    value = m_accessors.front ().second->_enumToValue (str, column);
  }
  return value;
}
std::string DataMerge::_valueToEnum (double value, size_t column) const
{
  std::string str;
  if (!m_row_merge)
  {
    boost::shared_ptr<DataAccessor> a;
    size_t c;
    accessor_offset_pair comp (column, boost::shared_ptr<DataAccessor>());
    accessor_offset_array::const_iterator bound =
      std::upper_bound (m_accessors.begin (), m_accessors.end (), comp, accessor_offset_pair_comp);

    a = bound->second;

    if (bound != m_accessors.begin ())
    {
      --bound;
      c = column - bound->first;
    }
    else
    {
      c = column;
    }
    str = a->_valueToEnum (value, c);
  }
  else
  {
    str = m_accessors.front ().second->_valueToEnum (value, column);
  }
  return str;
}

void DataMerge::deactivateEnums ()
{
  BOOST_FOREACH(accessor_offset_pair p, m_accessors)
  {
    p.second->deactivateEnums ();
  }
}

size_t DataMerge::getColumnCount () const
{
  if(m_accessors.empty())
    return 0;
  size_t c = m_accessors.size ();
  size_t offset = (!m_row_merge && c > 1) ? m_accessors[c - 2].first : 0;
  return offset + m_accessors.back ().second->getColumnCount ();
}
size_t DataMerge::getRowCount () const
{
  if(m_accessors.empty())
    return 0;
  size_t c = m_accessors.size ();
  size_t offset = (m_row_merge && c > 1) ? m_accessors[c - 2].first : 0;
  return offset + m_accessors.back ().second->getRowCount ();
}
double DataMerge::at (size_t row, size_t column) const
{
  size_t r, c;
  boost::shared_ptr<DataAccessor> a;
  accessor_offset_pair comp ((m_row_merge ? row : column), boost::shared_ptr<DataAccessor>());
  accessor_offset_array::const_iterator bound =
    std::upper_bound (m_accessors.begin (), m_accessors.end (), comp, accessor_offset_pair_comp);

  a = bound->second;

  if (bound != m_accessors.begin ())
  {
    --bound;
    if (m_row_merge)
    {
      c = column;
      r = row - bound->first;
    }
    else
    {
      c = column - bound->first;
      r = row;
    }
  }
  else
  {
    c = column;
    r = row;
  }
  return a->at (r, c);
}
std::string DataMerge::atEnum (size_t row, size_t column) const
{
  size_t r, c;
  boost::shared_ptr<DataAccessor> a;
  accessor_offset_pair comp ((m_row_merge ? row : column), boost::shared_ptr<DataAccessor>());
  accessor_offset_array::const_iterator bound =
    std::upper_bound (m_accessors.begin (), m_accessors.end (), comp, accessor_offset_pair_comp);

  a = bound->second;

  if (bound != m_accessors.begin ())
  {
    --bound;
    if (m_row_merge)
    {
      c = column;
      r = row - bound->first;
    }
    else
    {
      c = column - bound->first;
      r = row;
    }
  }
  else
  {
    c = column;
    r = row;
  }
  return a->atEnum (r, c);
}
void DataMerge::set(size_t row, size_t column, double value)
{
  size_t r, c;
  boost::shared_ptr<DataAccessor> a;
  accessor_offset_pair comp ((m_row_merge ? row : column), boost::shared_ptr<DataAccessor>());
  accessor_offset_array::const_iterator bound =
    std::upper_bound (m_accessors.begin (), m_accessors.end (), comp, accessor_offset_pair_comp);

  a = bound->second;

  if (bound != m_accessors.begin ())
  {
    --bound;
    if (m_row_merge)
    {
      c = column;
      r = row - bound->first;
    }
    else
    {
      c = column - bound->first;
      r = row;
    }
  }
  else
  {
    c = column;
    r = row;
  }
  a->set (r, c, value);
}
void DataMerge::set(size_t row, size_t column, const std::string& value)
{
  size_t r, c;
  boost::shared_ptr<DataAccessor> a;
  accessor_offset_pair comp ((m_row_merge ? row : column), boost::shared_ptr<DataAccessor>());
  accessor_offset_array::const_iterator bound =
    std::upper_bound (m_accessors.begin (), m_accessors.end (), comp, accessor_offset_pair_comp);

  a = bound->second;

  if (bound != m_accessors.begin ())
  {
    --bound;
    if (m_row_merge)
    {
      c = column;
      r = row - bound->first;
    }
    else
    {
      c = column - bound->first;
      r = row;
    }
  }
  else
  {
    c = column;
    r = row;
  }
  a->set (r, c, value);
}

DataAccessor::columnType DataMerge::getColumnType (size_t column) const
{
  DataAccessor::columnType type;
  if (!m_row_merge)
  {
    boost::shared_ptr<DataAccessor> a;
    size_t c;
    accessor_offset_pair comp (column, boost::shared_ptr<DataAccessor>());
    accessor_offset_array::const_iterator bound =
      //std::upper_bound (m_accessors.cbegin (), m_accessors.cend (), comp, accessor_offset_pair_comp); c++11
      std::upper_bound (m_accessors.begin (), m_accessors.end (), comp, accessor_offset_pair_comp);

    a = bound->second;

    if (bound != m_accessors.begin ())
    {
      --bound;
      c = column - bound->first;
    }
    else
    {
      c = column;
    }
    type = a->getColumnType (c);
  }
  else
  {
    type = m_accessors.front ().second->getColumnType (column);
  }
  return type;
}
std::string DataMerge::getColumnName (size_t column) const
{
  std::string name;
  if (!m_row_merge)
  {
    boost::shared_ptr<DataAccessor> a;
    size_t c;
    accessor_offset_pair comp (column, boost::shared_ptr<DataAccessor>());
    accessor_offset_array::const_iterator bound =
      //std::upper_bound (m_accessors.cbegin (), m_accessors.cend (), comp, accessor_offset_pair_comp); c++11
      std::upper_bound (m_accessors.begin (), m_accessors.end (), comp, accessor_offset_pair_comp);

    a = bound->second;

    if (bound != m_accessors.begin ())
    {
      --bound;
      c = column - bound->first;
    }
    else
    {
      c = column;
    }
    name = a->getColumnName (c);
  }
  else
  {
    name = m_accessors.front ().second->getColumnName (column);
  }
  return name;
}
