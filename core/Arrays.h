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

#ifndef PCLREF_ARRAYS_H_
#define PCLREF_ARRAYS_H_

#include <PclRefLIB.h>
#include <MersenneTwister.h>

namespace pclref
{
  /// Helper class for simple array manipulations
  class Arrays
  {
  public:
    /// pushes the natural numbers from min to max at end of interval
    template<typename T>
    static void natural (int min, int max, std::vector<T>& interval);
    /// pushes value count times at end of interval
    template<typename T>
    static void repeat (T value, size_t count, std::vector<T>& interval);
    /// pushes n random items from [min,max] at end of picks
    template<typename T>
    static void random (int min, int max, size_t n, std::vector<T>& picks);
    /// picks n items uniformly from [min,max] wo replacement and pushes them sorted at end of picks
    template<typename T>
    static void pick (int min, int max, size_t n, std::vector<T>& picks);

    /// permutes items of data
    template<typename T>
    static void shuffle (std::vector<T>& data);

    /// pushes n random items from data at end of picks
    template<typename T>
    static void random (const std::vector<T>& data, size_t n, std::vector<T>& picks);

    /// picks n items uniformly from data wo replacement and pushes them sorted at end of picks
    template<typename T>
    static void pick (const std::vector<T>& data, size_t n, std::vector<T>& picks);

    /// splits data into k partitions
    template<typename T>
    static void partition (const std::vector<T>& data, size_t k, std::vector<std::vector<T> >& partitions);

  private:
    DISALLOW_COPY_AND_ASSIGN (Arrays);
  };

  template<typename T>
  void Arrays::natural (int min, int max, std::vector<T>& interval)
  {
    for (int i = min; i < max; ++i)
      interval.push_back ((T)i);
  }
  template<typename T>
  void Arrays::repeat (T value, size_t count, std::vector<T>& interval)
  {
    for (size_t i = 0; i < count; ++i)
      interval.push_back (value);
  }
  template<typename T>
  void Arrays::random (int min, int max, size_t n, std::vector<T>& picks)
  {
    for (size_t i = 0; i < n; ++i)
      picks.push_back((T)((int)MersenneTwister::nextUint(0, max-min)-min));
  }
  template<typename T>
  void Arrays::pick (int min, int max, size_t n, std::vector<T>& picks)
  {
    int N = max-min;
    int looked_at = 0;
    size_t selected = 0;
    while(selected < n)
    {
      float u = MersenneTwister::next ();
      if((N - looked_at)*u < n - selected)
      {
        picks.push_back((T)(looked_at+min));
        ++selected;
      }
      ++looked_at;
    }
  }
  template<typename T>
  void Arrays::shuffle (std::vector<T>& data)
  {
    for (size_t i = data.size () - 1; i > 0; --i)
    {
      size_t j = MersenneTwister::nextUint (0, i);
      std::swap (data[i], data[j]);
    }
  }
  template<typename T>
  void Arrays::random (const std::vector<T>& data, size_t n, std::vector<T>& picks)
  {
    for (size_t i = 0; i < n; ++i)
      picks.push_back(data[MersenneTwister::next()*data.size()]);
  }
  template<typename T>
  void Arrays::pick (const std::vector<T>& data, size_t n, std::vector<T>& picks)
  {
    size_t N = data.size();
    size_t looked_at = 0;
    size_t selected = 0;
    while(selected < n)
    {
      float u = MersenneTwister::next ();
      if((N - looked_at)*u < n - selected)
      {
        picks.push_back(data[looked_at]);
        ++selected;
      }
      ++looked_at;
    }
  }
  template<typename T>
  void Arrays::partition (const std::vector<T>& data, size_t k, std::vector<std::vector<T> >& partitions)
  {
    partitions.resize (k);
    for (size_t i = 0; i < data.size (); ++i)
    {
      partitions[i % k].push_back (data[i]);
    }
  }
}


#endif /* ARRAYS_H_ */
