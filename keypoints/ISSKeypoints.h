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
 * Created on: May 27 2014
 *
 */

#ifndef ISS_KEYPOINTS_H_
#define ISS_KEYPOINTS_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class ISSKeypoints
  *
  * \brief Detects ISS keypoints
  *
  * \param iss_salient_radius : double (0.6)
  * \param iss_non_max_radius : double (0.4)
  * \param iss_gamma_21 : double (0.975)
  * \param iss_gamma_32 : double (0.975)
  * \param iss_min_neighbors : int (5)
  * \param iss_threads : int (4)
  *
  * ISS border detection can be additionally enabled
  *
   * \param iss_border_detection : bool (false)
   * \param iss_border_radius : double (0.1)
   * \param iss_normal_radius : double (0.4)
  *
  *
  * \pre ProcessingContext::PointCloud cloud
  * \pre ProcessingContext::NormalCloud normals
  * \post ProcessingContext::KeypointCloud keypoints
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class ISSKeypoints : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    ISSKeypoints();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_ISSKeypoints(T,K,F) template class pclref::ISSKeypoints <T,K,F>;

#endif
