/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2012/07/26 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: August 6 2014
 *
 */

#ifndef KEYPOINT_STATS_SELF_H_
#define KEYPOINT_STATS_SELF_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class KeypointStatsSelf
  *
  * \brief Creates histograms of the distances of random keypoint pairs
  *
  * This can be used in conjunction with UniquenessSelf
  *
  * \param ks_write_histos : bool (false)
  * \param keypoint_stats_k : int(5)
  *
  * \pre ProcessingContext::KeypointCloud keypoints
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class KeypointStatsSelf : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    KeypointStatsSelf();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_KeypointStatsSelf(T,K,F) template class pclref::KeypointStatsSelf <T,K,F>;

#endif
