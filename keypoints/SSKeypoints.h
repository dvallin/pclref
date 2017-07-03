/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2012/07/24 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: Sep 19 2014
 *
 */

#ifndef SS_KEYPOINTS_H_
#define SS_KEYPOINTS_H_

#include <PclRefLIB.h>

#ifndef PCLREF_LEAN
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class SSKeypoints
  *
  * \brief Detects SS keypoints
  *
  * \param ss_neighbourhood_constant : double (0.4)
  * \param ss_input_scale : double (1.0)
  *
  * \pre ProcessingContext::PointCloud cloud
  * \pre ProcessingContext::NormalCloud normals
  * \post ProcessingContext::KeypointCloud keypoints
  *
  * can also be run on clusters
  * \pre ProcessingContext::Clusters clusters
  */
  template<typename PointType, typename FeatureType>
  class SSKeypoints : public ProcessingStep<PointType, PointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, PointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, PointType, FeatureType> ContextType; ///< The Context

    SSKeypoints();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_SSKeypoints(T,F) template class pclref::SSKeypoints <T,F>;
#endif

#endif
