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

#ifndef UNIQUENESS_H_
#define UNIQUENESS_H_

#include <ProcessingStep.h>

namespace pclref
{

  /**
  * \class Uniqueness
  *
  * \brief Creates histograms of the first k nearest neighbor distances of keypoints from the overlap set
  *
  * The histograms are written to the additional logs
  *
  * \param uniqueness_write_histos : bool (false)
  * \param uniqueness_k : int(5)
  *
  * \pre ContextType::Transformation ground_truth_source
  * \pre ContextType::Transformation ground_truth_target
  * \pre ProcessingContext::PointCloud cloud_source
  * \pre ProcessingContext::KeypointCloud keypoints_target
  * \pre ProcessingContext::IndexCloud keypoints_overlap
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class Uniqueness : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    Uniqueness();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_Uniqueness(T,K,F) template class pclref::Uniqueness <T,K,F>;

#endif
