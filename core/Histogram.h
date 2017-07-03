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
 * Created on: July 30 2014
 *
 */

#ifndef PCLREF_HISTOGRAM_H_
#define PCLREF_HISTOGRAM_H_

#include <PclRefLIB.h>
#include <DataTable.h>

namespace pclref
{
  /**
  * \brief specialized DataTable representing a histogram
  */
  class Histogram : public DataTable
  {
  public:
    typedef boost::shared_ptr<Histogram> Ptr;

    /** \brief Constructor
      * \param min_value minimum bin value
      * \param max_value maximum bin value
      * \param bins bin count
      * \param clamp clamps values to the [min,max] range
      *
      * This Histogram creates several columns:
      * 1. n: the amount of values added
      * 2. mean: mean value of values in histogram range
      * 3. m2: variance*(n-1) values in histogram range
      * 4. mean_all: mean of all values
      * 5. m2_all: m2 of all values
      * 6. bin_lo: bin for all values smaller than min
      * now all bins
      * 7. bin_hi: bin for all values greater than max
      *
      * bins for point 6 and 7 are not created in clamping mode.
      */
    Histogram(double min_value, double max_value, size_t bins, bool clamp = false);

    /// add a new value
    void add(double value, size_t row = 0);

    /// set range of possible values.
    void setRange(double min, double max, bool clamp = false);

  protected:
    size_t m_idx_first_bin;
    size_t m_bins;
    double m_min;
    double m_max;
    bool m_clamp;
  };

}


#endif /* DATA_TABLE_H_ */
