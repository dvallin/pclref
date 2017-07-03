/*
 * ourcvfh_features.h
 *
 *  Created on: September 19, 2014
 *      Author: max
 */

#ifndef OUR_ourcvfh_FEATURES_H_
#define OUR_ourcvfh_FEATURES_H_

#include <PclRefLIB.h>

#ifndef PCLREF_LEAN
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class OURCVFHFeatures
  *
  * \brief Estimates OURCVFH features.
  *
  * \note This algorithm does not need any keypoint detection.
  *
  * \param ourcvfh_min_points int(50)
  * \param ourcvfh_cluster_tolerance double(0.4)
  * \param ourcvfh_normals_radius double(0.4)
  * \param ourcvfh_normalize_bins bool(false)
  * \param ourcvfh_curv_threshold double(0.03)
  * \param ourcvfh_eps_angle_threshold double(0.125)
  *
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::NormalCloud normals
  * \post ProcessingContext::FeatureCloud features
  * \post ProcessingContext::IndexCloud keypoint_indices
  */
  template<typename PointType, typename KeypointType>
  class OURCVFHFeatures : public ProcessingStep<PointType, KeypointType, OURCVFH_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, OURCVFH_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, OURCVFH_Type> ContextType; ///< The Context

    OURCVFHFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_OURCVFHFeatures(T,K) template class pclref::OURCVFHFeatures <T,K>;
#endif

#endif
