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

#ifndef KF_INFO_H_
#define KF_INFO_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class KFInfo
  *
  * \brief Creates Ids used by some Processing Steps
  *
  * \post pclref::identifier id
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class KFInfo : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    KFInfo();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_KFInfo(T,K,F) template class pclref::KFInfo <T,K,F>;

#endif
