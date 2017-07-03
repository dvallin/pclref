/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2014/07/31 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: August 1 2014
 *
 */

#ifndef T_INFO_H_
#define T_INFO_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class TInfo
  *
  * \brief Logs the ids point cloud pair
  *
  * \pre pclref::identifier source_id
  * \pre pclref::identifier target_id
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class TInfo : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    TInfo();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_TInfo(T,K,F) template class pclref::TInfo <T,K,F>;

#endif
