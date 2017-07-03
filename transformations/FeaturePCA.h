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
 * Created on: Oct 14 2014
 *
 */

#ifndef FEATURE_PCA_H_
#define FEATURE_PCA_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class FeaturePCA
  *
  * \brief Transforms feature spaces using PCA transfomation.
  *
  * \param pca_write : bool (false)
  * \param pca_whitened : bool (true)
  * \param pca_use_zca : bool (false)
  * \param pca_dimensions : int (0)
  *
  * There are two use cases for this class. Either a pca_stats structure is present in which case
  * a transformation should be applied to the current feature cloud. Or a transformation should be
  * estimated based on some feature clouds and written to the additional logs.
  *
  * For writing the trained data, pca_write must be set. If pca_dimensions is greater than zero
  * it gives the dimensionality of the desired space.
  *
  * To train a transformation
  * \pre bool accu
  * \pre ProcessingContext kfContexts
  *
  * each context in kfContexts:
  * \pre ProcessingContext::FeatureCloud features
  * \post Eigen::VectorXf pca_eigen_values
  * \post Eigen::MatrixXf pca_eigen_vectors
  * \post Eigen::VectorXf pca_mean
  *
  * To project features:
  * \pre DataAccessor pca_stats
  * \pre ProcessingContext::FeatureCloud features
  * \post ProcessingContext::FeatureCloud features
  *
  *
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class FeaturePCA : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    FeaturePCA();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

  private:
    void generateInputFeatureSpace(typename ContextType::Ptr context, typename ContextType::FeatureCloud::Ptr features);
    void projectFeatureSpaces(typename ContextType::Ptr context, typename ContextType::FeatureCloud::Ptr features);
  };
}

#define PCLREF_INSTANTIATE_FeaturePCA(T,K,F) template class pclref::FeaturePCA <T,K,F>;

#endif
