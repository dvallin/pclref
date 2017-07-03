/*
 * GTCorrespondences.h
 *
 *  Created on: Aug 15, 2014
 *      Author: max
 */

#ifndef G_T_CORRESPONDENCES_H_
#define G_T_CORRESPONDENCES_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class GTCorrespondences
  *
  * \brief Estimates Correspondences based on Ground Truth Distance of Keypoints
  *
  * \param gt_correspondences_random_sample : bool (true)
  * \param gt_correspondences_epsilon : float (0.5)
  *
  * \pre ProcessingContext::KeypointCloud keypoints_source
  * \pre ProcessingContext::Transformation ground_truth_source
  * \pre ProcessingContext::KeypointCloud keypoints_target
  * \pre ProcessingContext::Transformation ground_truth_target
  * \post pcl::Correspondences correspondences
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class GTCorrespondences : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    GTCorrespondences();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_GTCorrespondences(T,K,F) template class pclref::GTCorrespondences <T,K,F>;

#endif
