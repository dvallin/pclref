/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2012/07/24 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: September 5 2014
 *
 */

#ifndef UNIFORM_KEYPOINTS_H_
#define UNIFORM_KEYPOINTS_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class UniformKeypoints
  *
  * \brief Generates keypoints by downsampling the point cloud using a voxel grid filter
  *
  * \param uk_radius : double (0.4)
  *
  * \pre ProcessingContext::PointCloud cloud
  * \post ProcessingContext::KeypointCloud keypoints
  * \post ProcessingContext::IndexCloud keypoint_indices
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class UniformKeypoints : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    UniformKeypoints();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_UniformKeypoints(T,K,F) template class pclref::UniformKeypoints <T,K,F>;

#endif
