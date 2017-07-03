#include <Histogram.h>
using namespace pclref;

Histogram::Histogram(double min_value, double max_value, size_t bins, bool clamp)
  : m_bins(bins), m_min(min_value), m_max(max_value), m_clamp(clamp)
{
  m_max += std::numeric_limits<double>::epsilon();
  std::vector<DataAccessor::columnType> columnTypes;
  std::vector<std::string> columnNames;

  columnTypes.push_back(DataAccessor::FLOAT);
  columnTypes.push_back(DataAccessor::FLOAT);
  columnTypes.push_back(DataAccessor::FLOAT);
  columnTypes.push_back(DataAccessor::FLOAT);
  columnTypes.push_back(DataAccessor::FLOAT);
  columnNames.push_back("n");
  columnNames.push_back("mean");
  columnNames.push_back("m2");
  columnNames.push_back("mean_all");
  columnNames.push_back("m2_all");

  if(!clamp)
  {
    columnTypes.push_back(DataAccessor::FLOAT);
    columnNames.push_back("bin_lo");
  }
  m_idx_first_bin = columnNames.size();

  for(size_t b = 0; b < bins; ++b)
  {
    columnTypes.push_back(DataAccessor::FLOAT);
    columnNames.push_back("bin_" + precision_cast(b, 1));
  }
  if(!clamp)
  {
    columnTypes.push_back(DataAccessor::FLOAT);
    columnNames.push_back("bin_hi");
  }
  init(columnTypes, columnNames);
}

void Histogram::add(double value, size_t row)
{
  _ensure(row);

  double t = (value - m_min) / (m_max - m_min);
  size_t idx = 0;

  if(m_clamp) // clamp, so no explicit hi and lo bins present
  {
    idx = (t < 0.0 ? m_idx_first_bin :
           (t >= 1.0 ? m_bins + m_idx_first_bin - 1 :
            (size_t)(t * m_bins) + m_idx_first_bin));
  }
  else if(t >= 0.0 && t < 1.0)
  {
    idx = (size_t)(t * m_bins) + m_idx_first_bin;
  }
  else if(t < 0.0) // put this value in lo bin
  {
    idx = m_idx_first_bin - 1;
  }
  else // put this value in hi bin
  {
    idx = m_bins + m_idx_first_bin;
  }

  // increase n bin
  set(row, 0, at(row, 0) + 1.0);

  // only in range mean and m2
  if(idx >= m_idx_first_bin && idx < m_bins + m_idx_first_bin)
  {
    // update mean and m2 (wikipedia who cites knuth, who cites welford)
    double delta = value - at(row, 1);
    double mean = at(row, 1) + delta / at(row, 0);
    double m2 = at(row, 2) + delta * (value - mean);
    set(row, 1, mean);
    set(row, 2, m2);
  }
  {
    // all mean and m2
    // update mean and m2 (wikipedia who cites knuth, who cites welford)
    double delta = value - at(row, 3);
    double mean = at(row, 3) + delta / at(row, 0);
    double m2 = at(row, 4) + delta * (value - mean);
    set(row, 3, mean);
    set(row, 4, m2);
  }

  // update bin
  set(row, idx, at(row, idx) + 1.0);
}

void Histogram::setRange(double min, double max, bool clamp)
{
  m_min = min;
  m_max = max;
  m_clamp = clamp;
}
