/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2012/07/26 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: August 1 2014
 *
 */

#ifndef REPEATABILITY_H_
#define REPEATABILITY_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class Repeatability
  *
  * \brief Calculates the Repeatability measures
  *
  * \param gt_correspondences_epsilon : double (0.15)
  *
  * Repeatability can either be calculated over a range of epilon values (scale-invariant)
  * or for a single epsilon value.
  *
  * \param repeatability_range : bool (false)
  * \param repeatability_epsilon : double (1.5 * model_resolution)
  * \param repeatability_epsilon_min : double (0.5 * model_resolution)
  * \param repeatability_epsilon_max : double (3 * model_resolution)
  * \param repeatability_epsilon_step : double (0.5 * model_resolution)
  *
  * For unique Repeatability a scale value can be set, which governs the uniqueness radius
  * as a multiple of epsilon.
  *
  * \param repeatability_theta_scale : double (1.0)
  *
  * \pre ContextType::Transformation ground_truth_source
  * \pre ContextType::Transformation ground_truth_target
  * \pre ProcessingContext::PointCloud cloud_source
  * \pre ProcessingContext::PointCloud cloud_target
  * \pre ProcessingContext::KeypointCloud keypoints_source
  * \pre ProcessingContext::KeypointCloud keypoints_target
  *
  * \post ContextType::IndexCloud keypoints_rep
  * \post ContextType::IndexCloud keypoints_unique_rep
  * \post ContextType::IndexCloud keypoints_reciprocal_rep
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class Repeatability : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    Repeatability();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

  private:
    void calculateRepeatability(typename ContextType::IndexCloud::Ptr keypoints_overlap,
                                typename ContextType::IndexCloud::Ptr keypoint_indices_source,
                                typename pcl::search::KdTree<KeypointType>::Ptr keypoint_search,
                                typename pcl::search::KdTree<KeypointType>::Ptr keypoint_inv_search,
                                typename ContextType::PointCloud::Ptr cloud_trans_source,
                                typename ContextType::KeypointCloud::Ptr keypoints_trans_target,
                                double epsilon, double theta,
                                typename ContextType::IndexCloud::Ptr keypoints_rep,
                                double& scale_rep,
                                typename ContextType::IndexCloud::Ptr keypoints_unique_rep,
                                double& unique_scale_rep,
                                typename ContextType::IndexCloud::Ptr keypoints_reciprocal_rep,
                                double& reciprocal_scale_rep);
  };
}

#define PCLREF_INSTANTIATE_Repeatability(T,K,F) template class pclref::Repeatability <T,K,F>;

#endif
