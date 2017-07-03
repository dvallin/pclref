/*
 * vfh_features.h
 *
 *  Created on: September 17, 2014
 *      Author: max
 */

#ifndef USC_FEATURES_H_
#define USC_FEATURES_H_

#include <PclRefLIB.h>

#ifndef PCLREF_LEAN
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class USCFeatures
  *
  * \brief Estimates USC features.
  *
      * \note this may change the keypoint cloud
      *
  * \param usc_radius_search double(0.4)
  * \param usc_local_radius double(0.4)
  * \param usc_minimal_radius double(0.4)
  * \param usc_point_density_radius double(0.4)
  *
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::NormalCloud normals
  * \pre ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::KeypointCloud keypoints
  * \post ProcessingContext::FeatureCloud features
  */
  template<typename PointType, typename KeypointType>
  class USCFeatures : public ProcessingStep<PointType, KeypointType, USC_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, USC_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, USC_Type> ContextType; ///< The Context

    USCFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_USCFeatures(T,K) template class pclref::USCFeatures <T,K>;
#endif

#endif
