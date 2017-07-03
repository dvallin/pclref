/*
 * narf_features.h
 *
 *  Created on: May 29, 2014
 *      Author: max
 */

#ifndef NARF_FEATURES_H_
#define NARF_FEATURES_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class NARFFeatures
  *
  * \brief Estimates NARF features.
  *
  * \note this may change the keypoint cloud
  *
  * \param narf_features_support_size double(0.8)
  * \param narf_features_rotation_invariant bool(true)
  *
  * \pre ProcessingContext::RangeImage range_image
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::FeatureCloud features
  * \post ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::KeypointCloud keypoints
  */
  template<typename PointType, typename KeypointType>
  class NARFFeatures : public ProcessingStep<PointType, KeypointType, NARFF_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, NARFF_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, NARFF_Type> ContextType; ///< The Context

    NARFFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_NARFFeatures(T,K) template class pclref::NARFFeatures <T,K>;

#endif
