/*
 * estimate_correspondences.h
 *
 *  Created on: June 2, 2014
 *      Author: max
 */

#ifndef ICP_H_
#define ICP_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \brief Align two Pointclouds using the Iterative Closest Point algorithm
  *
  * if ransac score is present and smaller than ransac_score_threshold or icp_reject is true
  * ICP will not align (post conditions still hold).
  *
  * \param icp_remove_nans : bool (true)
  * \param icp_downsample : bool (false)
  * \param icp_leaf_size : float (0.05)
  * \param icp_reciprocal : bool (true)
  * \param icp_max_iterations : float (35)
  * \param icp_fitness_epsilon : float (1e-10)
  * \param icp_transformation_epsilon : float (1e-10)
  * \param icp_min_overlap_ratio : float (0.1)
  * \param icp_max_overlap_ratio : float (0.9)
  * \param icp_distance_threshold : float (model_resolution)
  * \param icp_ransac_inlier_threshold : float (model_resolution)
  *
  * \pre ProcessingContext::PointCloud cloud_source
  * \pre ProcessingContext::PointCloud cloud_target
  * \pre ProcessingContext::Transformation keypoint_transformation
  * \post ProcessingContext::PointCloud output_cloud
  * \post ProcessingContext::Transformation final_transformation
  *
  * Can also be run on a context containing an mstContext.
  * \pre ProcessingContext mstContext (a subset of ceContexts)
  * \see pclref::MST
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class ICP : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    ICP();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

  private:
    int doICP(typename ContextType::Ptr context, float& ms, float& score, float& trans, float& rot);
  };
}

#define PCLREF_INSTANTIATE_ICP(T,K,F) template class pclref::ICP <T,K,F>;

#endif

