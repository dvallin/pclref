/*
 * PointCloudStats.h
 *
 *  Created on: July 31, 2014
 *      Author: max
 */

#ifndef POINT_CLOUD_STATS_H_
#define POINT_CLOUD_STATS_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class PointCloudStats
  *
  * \brief Logs several statistics about a point cloud
  *
  * this class write the x, y and z coordinates of the two extremal points
  * of the axis aligned bounding box of the point cloud to the standard log.
  *
  * \pre ProcessingContext::PointCloud cloud
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class PointCloudStats : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    PointCloudStats();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_PointCloudStats(T,K,F) template class pclref::PointCloudStats <T,K,F>;

#endif
