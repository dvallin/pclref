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
 * Created on: May 22 2014
 *
 */

#ifndef CREATE_KEYPOINT_INDICES_H_
#define CREATE_KEYPOINT_INDICES_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class CreateKeypointIndices
  *
  * \brief Creates an IndexCloud from a KeypointCloud
  *
  * As some Keypoint algorithms give only a cloud of points, that are
  * unrelated to the PointCloud they are calculated from, this class creates
  * an IndexCloud using nearest neighbour queries of the keypoints in the
  * original PointCloud
  *
  * \pre ProcessingContext::PointCloud cloud
  * \pre ProcessingContext::KeypointCloud keypoints
  * \post ProcessingContext::IndexCloud keypoint_indices
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class CreateKeypointIndices : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    CreateKeypointIndices();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_CreateKeypointIndices(T,K,F) template class pclref::CreateKeypointIndices <T,K,F>;

#endif
