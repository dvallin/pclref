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
 * Created on: September 5 2014
 *
 */

#ifndef SIFT_KEYPOINTS_H_
#define SIFT_KEYPOINTS_H_

#include <PclRefLIB.h>

#ifndef PCLREF_LEAN
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class SIFTKeypoints
  *
  * \brief Detects SIFT keypoints
  *
  * Emulates intensity information either using the curvature or the distance to the sensor origin.
  *
  * \param sift_use_curvature : bool (true)
  * \param sift_min_contrast : double (0.001)
  * \param sift_min_scale : double (0.1)
  * \param sift_octaves : int (3)
  * \param sift_scales_per_octave : int (4)
  *
  * \pre ProcessingContext::PointCloud cloud
  * \pre ProcessingContext::NormalCloud normals
  * \post ProcessingContext::KeypointCloud keypoints
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class SIFTKeypoints : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    SIFTKeypoints();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_SIFTKeypoints(T,K,F) template class pclref::SIFTKeypoints <T,K,F>;
#endif

#endif
