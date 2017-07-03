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

#ifndef REMOVE_REMAP_H_
#define REMOVE_REMAP_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class RemoveRemap
  *
  * \brief Experimental: Removes the non-linear transform from the point cloud
  * \see pclref::RemapCloud
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class RemoveRemap : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    RemoveRemap();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_RemoveRemap(T,K,F) template class pclref::RemoveRemap <T,K,F>;

#endif
