/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2012/07/24 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: May 29 2014
 *
 */

#ifndef NARF_KEYPOINTS_H_
#define NARF_KEYPOINTS_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class NARFKeypoints
  *
  * \brief Detects NARF Keypoints
  *
  * \param narf_support_size : float (0.2f)
  *
  * \pre ProcessingContext::RangeImage range_image
  * \post ProcessingContext::IndexCloud keypoint_indices
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class NARFKeypoints : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    NARFKeypoints();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_NARFKeypoints(T,K,F) template class pclref::NARFKeypoints <T,K,F>;

#endif
