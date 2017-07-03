/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2012/10/28 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: Aug 27 2014
 *
 */

#ifndef REMAP_CLOUD_H_
#define REMAP_CLOUD_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class RemapCloud
  *
  * \brief Experimental: Applies a non-linear transform to the point cloud
  *
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class RemapCloud : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    RemapCloud();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_RemapCloud(T,K,F) template class pclref::RemapCloud <T,K,F>;

#endif
