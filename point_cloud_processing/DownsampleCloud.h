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
 * Created on: May 22 2014
 *
 */

#ifndef DOWNSAMPLE_CLOUD_H_
#define DOWNSAMPLE_CLOUD_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class DownsampleCloud
  *
  * \brief Voxelgrid based downsampling of PointCloud
  *
  * uses a Voxelgrid to downsample a PointCloud. Adjust the resolution of
  * the applied downsampling by adjusting leaf_size.
  *
  * \param leaf_size : double (0.01) sidelength of smallest cube in Voxelgrid
  *
  * \pre ProcessingContext::PointCloud cloud
  * \post ProcessingContext::PointCloud cloud
  *
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class DownsampleCloud : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    DownsampleCloud();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_DownsampleCloud(T,K,F) template class pclref::DownsampleCloud <T,K,F>;

#endif
