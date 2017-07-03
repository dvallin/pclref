/*
 * vfh_features.h
 *
 *  Created on: September 17, 2014
 *      Author: max
 */

#ifndef SC3D_FEATURES_H_
#define SC3D_FEATURES_H_

#include <PclRefLIB.h>

#ifndef PCLREF_LEAN
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class SC3DFeatures
  *
  * \brief Estimates 3D Shape Context features.
  *
  * \param sc3d_radius_search double(0.4)
  * \param sc3d_minimal_radius double(0.2)
  * \param sc3d_point_density_radius double(0.2)
  *
  * \pre ProcessingContext::PointCloud remapped_cloud or cloud
  * \pre ProcessingContext::NormalCloud normals
  * \pre ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::FeatureCloud features
  */
  template<typename PointType, typename KeypointType>
  class SC3DFeatures : public ProcessingStep<PointType, KeypointType, SC3D_Type>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, SC3D_Type> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, SC3D_Type> ContextType; ///< The Context

    SC3DFeatures();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_SC3DFeatures(T,K) template class pclref::SC3DFeatures <T,K>;
#endif

#endif
