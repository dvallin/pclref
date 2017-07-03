/*
 * EstimateNormals.h
 *
 *  Created on: May 22, 2014
 *      Author: max
 */

#ifndef ESTIMATE_NORMALS_H_
#define ESTIMATE_NORMALS_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class EstimateNormals
  *
  * \brief Estimate Normals of a PointCloud
  *
  * Calculates Normals of points of a PointCloud by standard PCA-based normal
  * estimation.
  *
  * \param radius_search : double (0.05) radius around each point to consider
  * \param remove_nans : bool (true) remove points whithout normal from cloud
  *
  * \pre ProcessingContext::PointCloud cloud
  * \post ProcessingContext::NormalCloud normals
  *
  * \note As normal estimation may fail (i.e. if radius_search is too
  * low) the resulting normals may contain NaNs for some points. In this case
  * consider setting remove_nans or increase radius_search.
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class EstimateNormals : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    EstimateNormals();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_EstimateNormals(T,K,F) template class pclref::EstimateNormals <T,K,F>;

#endif
