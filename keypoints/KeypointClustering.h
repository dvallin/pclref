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

#ifndef KEYPOINT_CLUSTERING_H_
#define KEYPOINT_CLUSTERING_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class KeypointClustering
  *
  * \brief Creates Clusters based on Keypoints
  *
  * \param kc_neighbours : int(1)
  * \param kc_keypoint_distance_threshold : double (std::numeric_limits<double>::max())
  *
  * \pre ProcessingContext::PointCloud cloud
  * \pre ProcessingContext::KeypointCloud keypoints
    * \post ProcessingContext::Clusters clusters
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class KeypointClustering : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    KeypointClustering();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_KeypointClustering(T,K,F) template class pclref::KeypointClustering <T,K,F>;

#endif
