/*
 * estimate_correspondences.h
 *
 *  Created on: June 2, 2014
 *      Author: max
 */

#ifndef ESTIMATE_CORRESPONDENCES_H_
#define ESTIMATE_CORRESPONDENCES_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class EstimateCorrespondences
  *
  * \brief Estimates Correspondences based on Feature Distance of Keypoints
  *
  * \param ec_reciprocal : bool (true) use reciprocal correspondence finding
  * \param ec_keypoint_distance_threshold : float (std::numeric_limits<double>::max()) thresholds keypoint distance
  * \param ec_neighbours : int (1) amount of considered neighbours
  *
  * Correspondence Log writing (if this creates too large log files, decrease max rows)
  * \param estimate_correspondences_write_sets : bool (false)
  * \param estimate_correspondences_write_features : bool (false)
  * \param ec_max_rows : int (9999)
  *
  * FLANN Parameters
  * \param feature_distance_metric : int (4) switches the distance metric
  * \arg 0 - L2
  * \arg 1 - ChiSquareDistance
  * \arg 2 - L1
  * \arg 3 - HistIntersectionDistance
  * \arg 4 - HellingerDistance
  * \param ec_flann_epsilon : double (0.0) the epsilon value expressing allowed imprecision
  * \param ec_flann_checks : int (256) the amount of checks (especially for MultiKDTree)
  * \param ec_flann_index_creator : int (1) the FLANN index creator (i.e. search structure) to be used
  *
  * possible values are:
  * \code
  * enum IndexCreator
  * {
  *      KDTree = 0,
  *      MultiKDTree = 1,
  *      KMeans = 2,
  *      Linear = 3,
  *     Composite = 4,
  *      Lsh = 5,
  *      Auto = 6
  *    };
  * \endcode
  *
  * depending on the index creator some of the following additional parameters are available:
  * ec_flann_trees, ec_flann_branching, ec_flann_iterations, ec_flann_centers, ec_flann_cb_index,
  * ec_flann_table_number, ec_flann_key_size, ec_flann_multi_probe_level, ec_flann_target_precision,
  * ec_flann_build_weight, ec_flann_memory_weight, ec_flann_sample_fraction.
  * You can look them up at the FLANN documentation, though standard parameters should be very good already.
  *
  * \pre ProcessingContext::FeatureCloud features_source
  * \pre ProcessingContext::FeatureCloud features_target
  * \post pcl::Correspondences correspondences
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class EstimateCorrespondences : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    EstimateCorrespondences();
    ~EstimateCorrespondences();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

  private:
    template<typename T>
    double analyze(typename ContextType::Ptr context,
                   boost::shared_ptr<pcl::PointCloud<T> > features_source,
                   boost::shared_ptr<pcl::PointCloud<T> > features_target,
                   boost::shared_ptr<pcl::Correspondences> correspondences);
    template<typename T>
    void estimate(typename ContextType::Ptr context,
                  boost::shared_ptr<pcl::PointCloud<T> > features_source,
                  boost::shared_ptr<pcl::PointCloud<T> > features_target,
                  boost::shared_ptr<pcl::Correspondences> correspondences);
    template<typename T, typename Metric>
    void estimate_with_metric(boost::shared_ptr<pcl::PointCloud<T> > features_source,
                              boost::shared_ptr<pcl::PointCloud<T> > features_target,
                              pcl::Correspondences& correspondences, double keypoint_distance_threshold, bool reciprocal, int neighbours);

#ifdef PCLREF_PARALLEL
    omp_lock_t m_writelock;
#endif
  };
}

#define PCLREF_INSTANTIATE_EstimateCorrespondences(T,K,F) template class pclref::EstimateCorrespondences <T,K,F>;

#endif
