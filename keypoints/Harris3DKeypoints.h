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

#ifndef HARRIS_3D_KEYPOINTS_H_
#define HARRIS_3D_KEYPOINTS_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class Harris3DKeypoints
  *
  * \brief Detects Harris3D Keypoints
  *
  * \param harris_method : int(0) switches the response method of Harris3D between
  * \arg 0 - Harris
  * \arg 1 - Noble
  * \arg 2 - Lowe
  * \arg 3 - Tomasi
  * \arg 4 - Curvature
  *
  * \param harris_non_max_supression : bool (true)
  * \param harris_radius : float (0.5f)
  * \param harris_method : int (0)
  *
  * \pre ProcessingContext::PointCloud cloud
  * \pre ProcessingContext::NormalCloud normals
  * \post ProcessingContext::KeypointCloud keypoints
  *
  * can also be run on clusters
  * \pre ProcessingContext::Clusters clusters
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class Harris3DKeypoints : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    Harris3DKeypoints();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_Harris3DKeypoints(T,K,F) template class pclref::Harris3DKeypoints <T,K,F>;

#endif
