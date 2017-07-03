/*
 * estimate_correspondences.h
 *
 *  Created on: June 2, 2014
 *      Author: max
 */

#ifndef MST_H_
#define MST_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class MST
  *
  * \brief Calculates a Maximum Weight Spanning tree of point cloud pairs based on the RANSAC score
  *
  * this will only work in the accumulation pipeline. So it gets a context of contexts and the accu
  * flag needs to be set! See Combiner for usage examples.
  *
  * creates a Processing Context of the contexts that are part of the spanning tree.
  *
  * \pre bool accu
  * \pre ProcessingContext ceContexts
  * Preconditions apply to each Processing Context in ceContexts:
  * \pre int source_id
  * \pre int target_id
  * \pre double ransac_score
  * \post ProcessingContext mstContext (a subset of ceContexts)
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class MST : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    MST();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_MST(T,K,F) template class pclref::MST <T,K,F>;

#endif

