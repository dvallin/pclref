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
 * Created on: August 1 2014
 *
 */

#ifndef PERSISTENCE_H_
#define PERSISTENCE_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class Persistence
  *
  * \brief Calculates the mean of the features and retains only persistent features.
  *
  * Distances to the mean feature can be written to the additional log
  *
  * \param pe_write_distances : bool(false)
  *
  * A feature outside of a range in standard deviations expressed by following parameters is removed:
  *
  * \param pe_min_sigma : double (0.0)
  * \param pe_max_sigma : double (std::numeric_limits<double>::max())
  *
  * \pre ProcessingContext::FeatureCloud features
  * \pre ProcessingContext::KeypointCloud keypoints_target
  * \pre ProcessingContext::IndexCloud keypoint_indices
  *
  * This class can change the following:
  *
  * \post ProcessingContext::FeatureCloud features
  * \post ProcessingContext::KeypointCloud keypoints_target
  * \post ProcessingContext::IndexCloud keypoint_indices
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class Persistence : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    Persistence();
    ~Persistence();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

  private:
    template<typename T>
    void analyze(boost::shared_ptr<pcl::PointCloud<T> >& features, boost::shared_ptr<pcl::PointCloud<T> >& features_out,
                 typename ContextType::KeypointCloud::Ptr& keypoints, typename ContextType::KeypointCloud::Ptr& keypoints_out,
                 typename ContextType::IndexCloud::Ptr& keypoint_indices, typename ContextType::IndexCloud::Ptr& keypoint_indices_out,
                 size_t run);

#ifdef PCLREF_PARALLEL
    omp_lock_t m_writelock;
#endif
  };
}

#define PCLREF_INSTANTIATE_Persistence(T,K,F) template class pclref::Persistence <T,K,F>;

#endif
