/*
 * vfh_features.h
 *
 *  Created on: September 17, 2014
 *      Author: max
 */

#ifndef SHOT_FEATURES_H_
#define SHOT_FEATURES_H_

#include <PclRefLIB.h>

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class SHOTFeatures
  *
  * \brief Estimates SHOT features.
  *
  * \note this may change the keypoint cloud
  *
  * \param shot_lrf_radius double(0.8)
  * \param shot_radius_search double(1.0)
  *
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::NormalCloud normals
  * \pre ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::KeypointCloud keypoints
  * \post ProcessingContext::FeatureCloud features
  */
  template<typename PointType, typename KeypointType>
  class SHOTFeatures : public ProcessingStep<PointType, KeypointType, SHOT_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, SHOT_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, SHOT_Type> ContextType; ///< The Context

    SHOTFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_SHOTFeatures(T,K) template class pclref::SHOTFeatures <T,K>;

#endif
