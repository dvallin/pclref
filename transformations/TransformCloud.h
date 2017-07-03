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
 * Created on: Aug 27 2014
 *
 */

#ifndef TRANSFORM_CLOUD_H_
#define TRANSFORM_CLOUD_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class TransformCloud
  *
  * \brief Linearly transforms the cloud
  *
  * \param tc_apply : bool (true)
  *
  * If tc_demean is active, the point cloud is demeaned.
  *
  * \param tc_demean : bool (true)
  *
  * else the point cloud can be randomly transformed:
  *
  * \param tc_dx : double(5)
  * \param tc_dy : double(5)
  * \param tc_dz : double(5)
  *
  * also some rotational axis can be locked:
  *
  * \param pitch_lock : bool (true)
  * \param yaw_lock : bool (false)
  * \param roll_lock : bool (true)
  *
  * \pre ProcessingContext::PointCloud cloud
  * \post ProcessingContext::PointCloud cloud
  *
  * \note If ground truth information is present, it will be updated accordingly! Note that this process cannot be reversed!
  *
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class TransformCloud : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    TransformCloud();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_TransformCloud(T,K,F) template class pclref::TransformCloud <T,K,F>;

#endif
