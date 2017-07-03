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
 * Created on: May 27 2014
 *
 */

#ifndef CONVERT_TO_RANGE_IMAGE_H_
#define CONVERT_TO_RANGE_IMAGE_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class ConvertToRangeImage
  *
  * \brief Converts a PointCloud to a RangImage
  *
  * This class converts a Cloud of three-dimensional points to a two-
  * dimensional range image. This process does implicit downsampling.
  *
  * \pre ProcessingContext::PointCloud cloud
  * \post ProcessingContext::RangeImage range_image
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class ConvertToRangeImage : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    ConvertToRangeImage();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_ConvertToRangeImage(T,K,F) template class pclref::ConvertToRangeImage <T,K,F>;

#endif
