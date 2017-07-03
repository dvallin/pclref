/*
 * GTCorrespondences.h
 *
 *  Created on: Aug 15, 2014
 *      Author: max
 */

#ifndef CORRESPONDENCE_ILIER_RATE_H_
#define CORRESPONDENCE_ILIER_RATE_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class CorrespondenceInlierRate
  *
  * \brief Calculates the the Inlier Rate of the Correspondence (CI and CM)
  *
  * This class calculates the CI and CM measure base on uniquely repeatable
  * keypoint pairs. Results are written to standard log.
  *
  * \param gt_correspondences_epsilon : double (0.15)
  *
  * \pre ContextType::IndexCloud keypoints_unique_rep
  * \pre ContextType::Transformation ground_truth_source
  * \pre ContextType::Transformation ground_truth_target
  * \pre ProcessingContext::PointCloud cloud_source
  * \pre ProcessingContext::PointCloud cloud_target
  * \pre ProcessingContext::KeypointCloud keypoints_source
  * \pre ProcessingContext::KeupointCloud keypoints_target
  * \pre pcl::Correspondences correspondences_unfiltered or correspondences
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class CorrespondenceInlierRate : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    CorrespondenceInlierRate();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_CorrespondenceInlierRate(T,K,F) template class pclref::CorrespondenceInlierRate <T,K,F>;

#endif
