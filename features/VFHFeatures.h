/*
 * vfh_features.h
 *
 *  Created on: September 3, 2014
 *      Author: max
 */

#ifndef VFH_FEATURES_H_
#define VFH_FEATURES_H_

#include <PclRefLIB.h>

#ifndef PCLREF_LEAN
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class VFHFeatures
  *
  * \brief Estimates VFH features.
  *
  * \param vfh_normalize_bins bool(true)
  * \param vfh_normalize_distance bool(false)
  * \param vfh_fill_size_component bool(true)
  *
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::NormalCloud normals
  * \pre ProcessingContext::Clusters clusters
  * \post ProcessingContext::FeatureCloud features
  */
  template<typename PointType, typename KeypointType>
  class VFHFeatures : public ProcessingStep<PointType, KeypointType, VFH_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, VFH_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, VFH_Type> ContextType; ///< The Context

    VFHFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_VFHFeatures(T,K) template class pclref::VFHFeatures <T,K>;
#endif

#endif
