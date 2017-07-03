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
 * Created on: July 25 2014
 *
 */

#ifndef CALCULATE_OVERLAP_H
#define CALCULATE_OVERLAP_H

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class CalculateOverlap
  *
  * \brief Calculates the Overlap set of a two clouds
  *
  * This class calculates the overlap set of two clouds by nearest neighbour
  * queries.
  *
  * \param overlap_on_clouds : bool (true) calculate overlap of point clouds
  * \param overlap_on_keypoints : bool (true) calculate overlap of features
  *
  * \pre ContextType::Transformation ground_truth_source
  * \pre ContextType::Transformation ground_truth_target
  * \post ProcessingContext::IndexCloud keypoints_overlap
  * \post ProcessingContext::IndexCloud cloud_overlap
  *
  * if overlap_on_clouds is true, the context must contain:
  * \pre ProcessingContext::PointCloud cloud_source
  * \pre ProcessingContext::PointCloud cloud_target
  *
  * if overlap_on_keypoints is true, the context must contain:
  * \pre ProcessingContext::KeypointCloud keypoints_source
  * \pre ProcessingContext::KeypointCloud keypoints_target
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class CalculateOverlap : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    CalculateOverlap();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_CalculateOverlap(T,K,F) template class pclref::CalculateOverlap <T,K,F>;

#endif
