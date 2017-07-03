/*
 * vfh_features.h
 *
 *  Created on: September 17, 2014
 *      Author: max
 */

#ifndef SI_FEATURES_H_
#define SI_FEATURES_H_

#include <PclRefLIB.h>

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class SIFeatures
  *
  * \brief Estimates Spin Image features.
  *
      * \note this may change the keypoint cloud
      *
  * \param si_image_width int(8)
  * \param si_min_points int(5)
  * \param si_support_angle_cos double(0.5)
  * \param si_radius_search double(0.4)
  * \param si_angular bool(false)
  *
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::NormalCloud normals
  * \pre ProcessingContext::KeypointCloud keypoints
  * \pre ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::FeatureCloud features
  * \post ProcessingContext::KeypointCloud keypoints
  * \post ProcessingContext::IndexCloud keypoint_indices
  */
  template<typename PointType, typename KeypointType>
  class SIFeatures : public ProcessingStep<PointType, KeypointType, SI_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, SI_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, SI_Type> ContextType; ///< The Context

    SIFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_SIFeatures(T,K) template class pclref::SIFeatures <T,K>;

#endif
