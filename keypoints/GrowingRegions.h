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
 * Created on: September 13 2014
 *
 */

#ifndef GROWING_REGIONS_H_
#define GROWING_REGIONS_H_

#include <PclRefLIB.h>

#ifndef PCLREF_LEAN
#include <ProcessingStep.h>

#include <pcl/segmentation/region_growing.h>

namespace pclref
{
  /**
  * \class GrowingRegions
  *
  * \brief Clustering using Growing Regions
  *
  * \param gr_min_cluster_size : int(50)
  * \param gr_max_cluster_size : int (100000)
  * \param gr_number_of_neighbours : int (30)
  * \param gr_smoothness_threshold : double (3.0 / 180.0 * PI)
  * \param gr_curvature_threshold : double (1.0)
  *
  * \pre ProcessingContext::PointCloud cloud
  * \pre ProcessingContext::NormalCloud normals
  * \post ProcessingContext::KeypointCloud keypoints
  * \post ProcessingContext::IndexCloud keypoint_indices
    * \post ProcessingContext::Clusters clusters
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class GrowingRegions : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    GrowingRegions();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_GrowingRegions(T,K,F) template class pclref::GrowingRegions <T,K,F>;
#endif

#endif
