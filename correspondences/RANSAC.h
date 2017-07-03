/*
 * estimate_correspondences.h
 *
 *  Created on: June 2, 2014
 *      Author: max
 */

#ifndef RANSAC_H_
#define RANSAC_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class Ransac
  * \brief Applies a RanSAC Method on a set of correspondences
  * in order to find a good transformation between the two point cloud. The PCL
  * SAC model can be used or a general method based operating on correspondences directly.
  *
  * The RANSAC algorithm results in a keypoint transformation and filters
  * the set of correspondences. The original set is retained as
  * correspondences_unfiltered.
  *
  * \param ransac_refine_model : bool (false)
  * \param ransac_inlier_threshold : float (1.5)
  * \param ransac_max_iterations : int (1000)
  * \param ransac_method : int (2)
  * \param ransac_general_model : bool (false)
  * \arg 0 - pcl::SAC_RANSAC
  * \arg 1 - pcl::SAC_LMEDS
  * \arg 2 - pcl::SAC_MSAC
  * \arg 3 - pcl::SAC_RRANSAC
  * \arg 4 - pcl::SAC_RMSAC
  * \arg 5 - pcl::SAC_MLESAC
  * \arg 6 - pcl::SAC_PROSAC
  *
  * \pre ProcessingContext::KeypointCloud keypoints_source
  * \pre ProcessingContext::KeypointCloud keypoints_target
  * \pre pcl::Correspondences correspondences
  * \post ProcessingContext::Transformation keypoint_transformation
  * \post pcl::Correspondences correspondences_unfiltered
  * \post pcl::Correspondences correspondences
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class Ransac : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    Ransac();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_Ransac(T,K,F) template class pclref::Ransac <T,K,F>;

#endif
