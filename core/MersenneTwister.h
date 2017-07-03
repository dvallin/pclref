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

#ifndef PCLREF_MERSENNE_TWISTER_H_
#define PCLREF_MERSENNE_TWISTER_H_

#include <PclRefLIB.h>

namespace pclref
{
  /**
  * \brief a MersenneTwister pseudo random number generator
  */
  class MersenneTwister
  {
  public:
    /// standard seed = 5489UL
    static void setSeed (uint32_t seed);

    /// random uint32_t in [0, std::numerical_limits<uint32_t>::max()]
    static uint32_t nextUint ();
    /// random uint32_t in [min, max]
    static uint32_t nextUint (uint32_t min, uint32_t max);
    /// random float in [0, 1]
    static float next ();
    /// random float in [min, max]
    static float next (float min, float max);

  private:
    DISALLOW_COPY_AND_ASSIGN (MersenneTwister);

#define MT_ARRAY_SIZE 624
    static uint32_t seed_;
    static int index_;
    static uint32_t y_[MT_ARRAY_SIZE];

  };

}


#endif /* MERSENNE_TWISTER_H_ */
