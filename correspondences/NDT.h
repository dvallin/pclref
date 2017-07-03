/*
 * EstimateNormals.h
 *
 *  Created on: May 22, 2014
 *      Author: max
 */

#ifndef NDT_H_
#define NDT_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class NDT
  *
  * \brief Align two Pointclouds using the Normal Distribution Transform algorithm
  *
  * \param ndt_remove_nans : bool (true)
  * \param ndt_downsample : bool (false)
  * \param ndt_leaf_size : float (0.05)
  * \param ndt_max_iterations : float (35)
  * \param ndt_transformation_epsilon : float (1e-10)
  * \param ndt_resolution : float (10*model_resolution)
  * \param ndt_step_size : float (0.1)
  *
  * \pre ProcessingContext::PointCloud cloud_source
  * \pre ProcessingContext::PointCloud cloud_target
  * \pre ProcessingContext::Transformation keypoint_transformation
  * \post ProcessingContext::PointCloud output_cloud
  * \post ProcessingContext::Transformation final_transformation
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class NDT : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    NDT();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_NDT(T,K,F) template class pclref::NDT <T,K,F>;

#endif
