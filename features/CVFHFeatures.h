/*
 * vfh_features.h
 *
 *  Created on: September 17, 2014
 *      Author: max
 */

#ifndef CVFH_FEATURES_H_
#define CVFH_FEATURES_H_

#include <ProcessingStep.h>

#ifndef PCLREF_LEAN

namespace pclref
{
  /**
  * \class CVFHFeatures
  *
  * \brief Estimates CVFH features.
  *
  * \note This algorithm does not need any keypoint detection.
  *
  * \param cvfh_min_points int(50)
  * \param cvfh_cluster_tolerance double(0.4)
  * \param cvfh_normals_radius double(0.4)
  * \param cvfh_normalize_bins bool(false)
  * \param cvfh_curv_threshold double(0.03)
  * \param cvfh_eps_angle_threshold double(0.125)
  *
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::NormalCloud normals
  * \post ProcessingContext::FeatureCloud features
  * \post ProcessingContext::IndexCloud keypoint_indices
  */
  template<typename PointType, typename KeypointType>
  class CVFHFeatures : public ProcessingStep<PointType, KeypointType, CVFH_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, CVFH_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, CVFH_Type> ContextType; ///< The Context

    CVFHFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_CVFHFeatures(T,K) template class pclref::CVFHFeatures <T,K>;
#endif

#endif
