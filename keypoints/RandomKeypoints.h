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
 * Created on: Aug 15 2014
 *
 */

#ifndef RANDOM_KEYPOINTS_H_
#define RANDOM_KEYPOINTS_H_

#include <PclRefLIB.h>

#ifndef PCLREF_LEAN
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class RandomKeypoints
  *
  * \brief Generates keypoints by randomly sampling them
  *
  * \pre ProcessingContext::PointCloud cloud
  * \post ProcessingContext::KeypointCloud keypoints
  * \post ProcessingContext::IndexCloud keypoint_indices
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class RandomKeypoints : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    RandomKeypoints();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_RandomKeypoints(T,K,F) template class pclref::RandomKeypoints <T,K,F>;
#endif

#endif
