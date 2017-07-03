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

#ifndef SAMPLE_M_U_H_
#define SAMPLE_M_U_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class SampleMU
  *
  * \brief Creates the M and U sets
  *
  * The feature distances are written to the additional log if mu_write_sets is true,
  * if mu_write_features is also true the feature pairs are written. Note that the latter
  * might create very large log files. If this is the case, decrease mu_max_rows.
  *
  * The feature differences are also retained.
  *
  * \param m_u_epsilon : double (2*model_resolution)
  * \param mu_write_sets : bool (false)
  * \param mu_write_features : bool (false)
  * \param mu_max_rows : int(9999)
  *
  * \pre ContextType::Transformation ground_truth_source
  * \pre ContextType::Transformation ground_truth_target
  * \pre ProcessingContext::PointCloud cloud_source
  * \pre ProcessingContext::KeypointCloud keypoints_target
  * \pre ProcessingContext::IndexCloud keypoints_overlap
  * \pre ProcessingContext::FeatureCloud features_source
  * \pre ProcessingContext::FeatureCloud features_target
  *
  * \post ContextType::FeatureCloud match_feature_diffs
  * \post ContextType::FeatureCloud unmatch_feature_diffs
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class SampleMU : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    SampleMU();
    ~SampleMU();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

  private:
    template<typename T>
    void analyze(boost::shared_ptr<pcl::PointCloud<T> > features_source,
                 boost::shared_ptr<pcl::PointCloud<T> > features_target,
                 boost::shared_ptr<pcl::Correspondences> m_set,
                 boost::shared_ptr<pcl::Correspondences> u_set);

    template<typename T>
    void writeFeaturePair(size_t row, const T& f_source, const T& f_target, bool write_features);

#ifdef PCLREF_PARALLEL
    omp_lock_t m_writelock;
#endif
  };
}

#define PCLREF_INSTANTIATE_SampleMU(T,K,F) template class pclref::SampleMU <T,K,F>;

#endif
