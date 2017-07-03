/*
 * create_indices_from_keypoints.h
 *
 *  Created on: June 9, 2014
 *      Author: max
 */

#ifndef CREATE_KEYPOINTS_FROM_INDICES_H_
#define CREATE_KEYPOINTS_FROM_INDICES_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class CreateKeypointsFromIndices
  *
  * \brief Creates a KeypointCloud from an IndexCloud of Keypoints
  *
  * As some Keypoint algorithms give only an IndexCloud this class extracts
  * the actual points as a KeypointCloud
  *
  * \pre ProcessingContext::PointCloud cloud
  * \pre ProcessingContext::IndexCloud keypoint_indices
  * \post ProcessingContext::KeypointCloud keypoints
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class CreateKeypointsFromIndices : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    CreateKeypointsFromIndices();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_CreateKeypointsFromIndices(T,K,F) template class pclref::CreateKeypointsFromIndices <T,K,F>;

#endif
