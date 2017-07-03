/*
 * vfh_features.h
 *
 *  Created on: September 3, 2014
 *      Author: max
 */

#ifndef ESF_FEATURES_H_
#define ESF_FEATURES_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

#ifndef PCLREF_LEAN
namespace pclref
{
  /**
  * \class ESFFeatures
  *
  * \brief Estimates ESF features.
  *
  * \param esf_grid_size int(32)
  * \param esf_sample_factor double(0.1)
  *
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::NormalCloud normals
  * \pre ProcessingContext::Clusters clusters
  * \post ProcessingContext::FeatureCloud features
  */
  template<typename PointType, typename KeypointType>
  class ESFFeatures : public ProcessingStep<PointType, KeypointType, ESF_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, ESF_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, ESF_Type> ContextType; ///< The Context

    ESFFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_ESFFeatures(T,K) template class pclref::ESFFeatures <T,K>;
#endif

#endif
