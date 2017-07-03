/*
 * m_datatable.cc
 *
 *  Created on: May 19, 2014
 *      Author: max
 */

#include <DataTable.h>
#include <Csv.h>
using namespace pclref;

DataTable::DataTable ()
{
#ifdef PCLREF_PARALLEL
  omp_init_lock(&m_writelock);
#endif
}
DataTable::DataTable (const std::vector<DataAccessor::columnType>& columnTypes,
                      const std::vector<std::string>& columnNames)
{
#ifdef PCLREF_PARALLEL
  omp_init_lock(&m_writelock);
#endif
  init(columnTypes, columnNames);
}
DataTable::~DataTable ()
{
#ifdef PCLREF_PARALLEL
  omp_destroy_lock(&m_writelock);
#endif
}

void DataTable::init(const std::vector<DataAccessor::columnType>& columnTypes,
                     const std::vector<std::string>& columnNames)
{
  m_column_count = columnTypes.size();
  m_column_types.assign(columnTypes.begin(), columnTypes.end());
  m_column_names.assign(columnNames.begin(), columnNames.end());

  identifier enumid = FIRST_ID;
  m_enum_ids.resize (m_column_count);
  for (size_t i = 0; i < m_column_count; ++i)
  {
    if (m_column_types[i] == FLOAT)
    {
      m_enum_ids[i] = INVALID_ID;
    }
    else if (m_column_types[i] == ENUM)
    {
      m_enum_ids[i] = enumid++;
    }
  }
  m_enum_to_value_converters.resize (enumid);
  m_value_to_enum_converters.resize (enumid);

  m_row_count = 0;
}

void DataTable::fromCSV (const std::string& filename, bool has_header, char seperator)
{
  std::ifstream file(filename.c_str());
  fromCSV(file, has_header, seperator);
  file.close();
}
void DataTable::fromCSV (std::ifstream& file, bool has_header, char seperator)
{

  CSVIterator iter (file, seperator);
  CSVIterator iend;

  if (iter == iend)
    return;

  // init some stuff
  {
    CSVRow row = *iter;
    m_column_count = row.size ();

    m_column_names.resize (m_column_count);
    m_column_types.resize (m_column_count);
    m_enum_ids.resize (m_column_count);
  }

  // first line might hold columnType and name. read them in.
  if(has_header)
  {
    CSVRow row = *iter;
    identifier enumid = FIRST_ID;
    for (size_t i = 0; i < m_column_count; ++i)
    {
      std::string cell = row[i];
      std::string::size_type idx = cell.find_first_of(":", 0);

      std::string type;
      if(std::string::npos != idx)
      {
        m_column_names[i] = cell.substr(0, idx);
        type = cell.substr(idx+1, 1);
      }
      else
      {
        m_column_names[i] = cell;
        type = "f";
      }

      if (type == "e")
      {
        m_column_types[i] = ENUM;
        m_enum_ids[i] = enumid++;
      }
      else
      {
        m_column_types[i] = FLOAT;
        m_enum_ids[i] = INVALID_ID;
      }
    }

    m_enum_to_value_converters.resize (enumid);
    m_value_to_enum_converters.resize (enumid);
    ++iter;
  }
  // no header given, so this stupid implementation expects floating point values everywhere
  else
  {
    for (size_t i = 0; i < m_column_count; ++i)
    {
      m_column_names[i] = "NA";
      m_column_types[i] = FLOAT;
    }
  }

  m_row_count = 0;
  for (; iter != iend; ++iter)
  {
    CSVRow row = *iter;

    if (row.size () == m_column_count)
    {
      for (size_t i = 0; i < m_column_count; ++i)
      {
        double value;
        switch (m_column_types[i])
        {
        case FLOAT:
        {
          // tried boost::lexical_cast<double> here but it seems
          // to restrictive for some reason
          value = ::atof(row[i].c_str());
          break;
        }
        case ENUM:
        {
          enum_to_value& converter = m_enum_to_value_converters[m_enum_ids[i]];
          value_to_enum& inv_converter = m_value_to_enum_converters[m_enum_ids[i]];
          enum_to_value::iterator iter, iend;
          iter = converter.find (row[i]);
          iend = converter.end ();

          if (iter == iend)
          {
            value = (double)converter.size ();
            converter.insert (enum_to_value::value_type (row[i], value));
            inv_converter.insert (value_to_enum::value_type (value, row[i]));
          }
          else
          {
            value = iter->second;
          }
          break;
        }
        }

        m_data.push_back (value);
      }

      m_row_count++;
    }
  }
}
void DataTable::fromData (const std::vector<double> data, size_t rows, size_t cols)
{
  m_row_count = rows;
  m_column_count = cols;

  m_column_names.resize(m_column_count);
  m_column_types.resize (m_column_count);
  m_enum_ids.resize (m_column_count);

  for (size_t i = 0; i < m_column_count; ++i)
  {
    m_column_names[i] = "unnamed";
    m_column_types[i] = FLOAT;
    m_enum_ids[i] = INVALID_ID;
  }

  m_data.assign (data.begin (), data.end ());
}

#ifdef PCLREF_OPENCV
void DataTable::fromCvMat (cv::Mat& mat)
{
  m_column_count = mat.cols;
  m_row_count = mat.rows;

  m_column_names.resize(m_column_count);
  m_column_types.resize (m_column_count);
  m_enum_ids.resize (m_column_count);

  for (size_t i = 0; i < m_column_count; ++i)
  {
    m_column_names[i] = "unnamed";
    m_column_types[i] = FLOAT;
    m_enum_ids[i] = INVALID_ID;
  }

  m_data.resize (m_column_count*m_row_count);
  for (size_t j = 0; j < m_row_count; ++j)
  {
    for (size_t i = 0; i < m_column_count; ++i)
    {
      m_data[j*m_column_count + i] = mat.at<float> (j, i);
    }
  }
}
#endif

size_t DataTable::_getEnumSize (size_t column) const
{
  return m_enum_to_value_converters[m_enum_ids[column]].size ();
}
double DataTable::_enumToValue (const std::string& str, size_t column) const
{
  enum_to_value converter = m_enum_to_value_converters[m_enum_ids[column]];
  return converter[str];
}
std::string DataTable::_valueToEnum (double value, size_t column) const
{

  value_to_enum converter = m_value_to_enum_converters[m_enum_ids[column]];
  return converter[value];
}

void DataTable::deactivateEnums ()
{
  for (size_t i = 0; i < m_column_count; ++i)
  {
    m_column_types[i] = FLOAT;
    m_enum_ids[i] = INVALID_ID;
  }
}

size_t DataTable::getColumnCount () const
{
  return m_column_count;
}
size_t DataTable::getRowCount () const
{
  return m_row_count;
}
double DataTable::at (size_t row, size_t column) const
{
  return m_data[row*m_column_count + column];
}
std::string DataTable::atEnum (size_t row, size_t column) const
{
  return _valueToEnum(m_data[row*m_column_count + column], column);
}
void DataTable::set(size_t row, size_t column, double value)
{
  assert(m_column_types[column] != DataAccessor::ENUM);
  _ensure(row);


#ifdef PCLREF_PARALLEL
  omp_set_lock(&m_writelock);
#endif
  m_data[row*m_column_count + column] = value;
#ifdef PCLREF_PARALLEL
  omp_unset_lock(&m_writelock);
#endif
}
void DataTable::set(size_t row, size_t column, const std::string& value)
{
  assert(m_column_types[column] == DataAccessor::ENUM);
  _ensure(row);

  enum_to_value& converter = m_enum_to_value_converters[m_enum_ids[column]];
  value_to_enum& inv_converter = m_value_to_enum_converters[m_enum_ids[column]];
  enum_to_value::iterator iter, iend;
  iter = converter.find (value);
  iend = converter.end ();

#ifdef PCLREF_PARALLEL
  omp_set_lock(&m_writelock);
#endif
  double v;
  if (iter == iend)
  {
    v = (double)converter.size ();
    converter.insert (enum_to_value::value_type (value, v));
    inv_converter.insert (value_to_enum::value_type (v, value));
  }
  else
  {
    v = iter->second;
  }
  m_data[row*m_column_count + column] = v;
#ifdef PCLREF_PARALLEL
  omp_unset_lock(&m_writelock);
#endif
}
void DataTable::_ensure(size_t row)
{
#ifdef PCLREF_PARALLEL
  omp_set_lock(&m_writelock);
#endif
  if(row >= m_row_count)
  {
    _expandBy(row - m_row_count + 1, false);
  }
#ifdef PCLREF_PARALLEL
  omp_unset_lock(&m_writelock);
#endif
}
void DataTable::_expandBy(size_t rows, bool locking)
{
#ifdef PCLREF_PARALLEL
  if(locking) omp_set_lock(&m_writelock);
#endif
  {
    while(rows-- > 0)
    {
      ++m_row_count;
      BOOST_FOREACH(DataAccessor::columnType type, m_column_types)
      {
        if(type == FLOAT)
        {
          m_data.push_back(0.0);
        }
        else if(type == ENUM)
        {
          m_data.push_back(INVALID_ID);
        }
      }
    }
  }
#ifdef PCLREF_PARALLEL
  if(locking) omp_unset_lock(&m_writelock);
#endif
}

DataAccessor::columnType DataTable::getColumnType (size_t column) const
{
  return m_column_types[column];
}
std::string DataTable::getColumnName (size_t column) const
{
  return m_column_names[column];
}
