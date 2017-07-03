/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2014/9/13 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: September 15 2014
 *
 */

#ifndef KEYPOINT_NEIGHBORHOOD_H_
#define KEYPOINT_NEIGHBORHOOD_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class KeypointNeighborhood
  *
  * \brief Creates Clusters based on Keypoints using their scale if present
  *
  * \param kn_radius : double (0.4)
  * \param kn_extract_scale : bool (true)
  *
  * \pre ProcessingContext::PointCloud cloud
  * \pre ProcessingContext::KeypointCloud keypoints
    * \post ProcessingContext::Clusters clusters
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class KeypointNeighborhood : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    KeypointNeighborhood();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_KeypointNeighborhood(T,K,F) template class pclref::KeypointNeighborhood <T,K,F>;

#endif
