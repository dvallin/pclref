/*
 * mersenne_twister.cc
 *
 *  Created on: May 19, 2014
 *      Author: max
 */

#include <MersenneTwister.h>
using namespace pclref;

uint32_t MersenneTwister::seed_ = 5489UL;
int MersenneTwister::index_ = MT_ARRAY_SIZE + 1;
uint32_t MersenneTwister::y_[MT_ARRAY_SIZE];

void MersenneTwister::setSeed (uint32_t seed)
{
  seed_ = seed;
  index_ = MT_ARRAY_SIZE + 1;
}

uint32_t MersenneTwister::nextUint ()
{
#define M     397
#define HI    0x80000000
#define LO    0x7fffffff
  static const uint32_t A[2] = { 0, 0x9908b0df };
  uint32_t  e;

  if (index_ > MT_ARRAY_SIZE)
  {
    int i;
    y_[0] = seed_;

    for (i = 1; i<MT_ARRAY_SIZE; ++i)
    {
      y_[i] = (1812433253UL * (y_[i - 1] ^ (y_[i - 1] >> 30)) + i);
      /* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
      /* In the previous versions, MSBs of the seed affect   */
      /* only MSBs of the array mt[].                        */
      /* 2002/01/09 modified by Makoto Matsumoto             */
    }
  }

  if (index_ >= MT_ARRAY_SIZE)
  {
    int i;
    uint32_t h;

    for (i = 0; i<MT_ARRAY_SIZE - M; ++i)
    {
      h = (y_[i] & HI) | (y_[i + 1] & LO);
      y_[i] = y_[i + M] ^ (h >> 1) ^ A[h & 1];
    }
    for (; i<MT_ARRAY_SIZE - 1; ++i)
    {
      h = (y_[i] & HI) | (y_[i + 1] & LO);
      y_[i] = y_[i + (M - MT_ARRAY_SIZE)] ^ (h >> 1) ^ A[h & 1];
    }

    h = (y_[MT_ARRAY_SIZE - 1] & HI) | (y_[0] & LO);
    y_[MT_ARRAY_SIZE - 1] = y_[M - 1] ^ (h >> 1) ^ A[h & 1];
    index_ = 0;
  }

  e = y_[index_++];
  /* Tempering */
  e ^= (e >> 11);
  e ^= (e << 7) & 0x9d2c5680;
  e ^= (e << 15) & 0xefc60000;
  e ^= (e >> 18);

  return e;
#undef N
#undef M
#undef HI
#undef LO
}

uint32_t MersenneTwister::nextUint (uint32_t min, uint32_t max)
{
  double v = static_cast<double>(nextUint ());
  v /= static_cast<double>(std::numeric_limits<uint32_t>::max ());
  return static_cast<uint32_t>(min + (max - min)*v);
}
float MersenneTwister::next ()
{
  double v = static_cast<double>(nextUint ());
  v /= static_cast<double>(std::numeric_limits<uint32_t>::max ());
  return static_cast<float>(v);
}
float MersenneTwister::next (float min, float max)
{
  double v = static_cast<double>(nextUint ());
  v /= static_cast<double>(std::numeric_limits<uint32_t>::max ());
  return static_cast<float>(min + (max - min)*v);
}
