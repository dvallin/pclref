/*
 * estimate_correspondences.h
 *
 *  Created on: June 2, 2014
 *      Author: max
 */

#ifndef PREDICT_CORRESPONDENCES_H_
#define PREDICT_CORRESPONDENCES_H_

#ifdef PCLREF_OPENCV
#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class PredictCorrespondences
  * \brief Experimental Correspondence Estimation.
  * The idea is to use OpenCV machine learning algorithms trained directly on feature pairs.
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class PredictCorrespondences : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    PredictCorrespondences();
    ~PredictCorrespondences();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

#ifdef PCLREF_PARALLEL
    omp_lock_t m_writelock;
#endif

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
  };
}

#define PCLREF_INSTANTIATE_PredictCorrespondences(T,K,F) template class pclref::PredictCorrespondences <T,K,F>;
#endif
#endif
