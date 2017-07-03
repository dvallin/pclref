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
 * Created on: August 4 2014
 *
 */

#ifndef UNIQUENESS_SELF_H_
#define UNIQUENESS_SELF_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class UniquenessSelf
  *
  * \brief Creates histograms of the first k nearest neighbor distances of keypoints
  *
  * The histograms are written to the additional logs
  *
  * \param uniqueness_self_write_histos : bool (false)
  * \param uniqueness_k : int(5)
  *
  * \pre ProcessingContext::KeypointCloud keypoints
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class UniquenessSelf : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    UniquenessSelf();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_UniquenessSelf(T,K,F) template class pclref::UniquenessSelf <T,K,F>;

#endif
