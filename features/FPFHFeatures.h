/*
 * fpfh_features.h
 *
 *  Created on: May 29, 2014
 *      Author: max
 */

#ifndef FPFH_FEATURES_H_
#define FPFH_FEATURES_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class FPFHFeatures
  *
  * \brief Estimates FPFH features.
  *
  * \param fpfh_k_search int(50)
  * \param fpfh_radius_search double(0.8)
  *
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::NormalCloud normals
  * \pre ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::FeatureCloud features
  */
  template<typename PointType, typename KeypointType>
  class FPFHFeatures : public ProcessingStep<PointType, KeypointType, FPFH_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FPFH_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FPFH_Type> ContextType; ///< The Context

    FPFHFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_FPFHFeatures(T,K) template class pclref::FPFHFeatures <T,K>;

#endif
