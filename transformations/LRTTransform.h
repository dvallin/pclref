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
 * Created on: Oct 24 2014
 *
 */

#ifndef LRT_TRANSFORM_H_
#define LRT_TRANSFORM_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class LRTTransform
  *
  * \brief Transforms features using the likelihood ratio test transformation described in <a href="http://link.springer.com/chapter/10.1007/978-3-642-13408-1_18">Bosse et al. 2010</a>
  *
  * \param lrt_write : bool (false)
  *
  * There are two use cases for this class. Either a lrt_stats structure is present in which case
  * a transformation should be applied to the current feature cloud. Or a transformation should be
  * estimated based on some feature clouds and written to the additional logs.
  *
  * For writing the trained data, lrt_write must be set.
  *
  * To train a transformation
  * \pre bool accu
  * \pre ProcessingContext kfContexts
  *
  * each context in kfContexts:
  * \pre ProcessingContext::FeatureCloud features
  * \post Eigen::MatrixXf lrt_matrix
  *
  * To project features:
  * \pre DataAccessor lrt_stats
  * \pre ProcessingContext::FeatureCloud features
  * \post ProcessingContext::FeatureCloud features
  *
  *
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class LRTTransform : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    LRTTransform();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

  private:
    void generateInputFeatureSpace(typename ContextType::Ptr context,
                                   typename ContextType::FeatureCloud::Ptr matches,
                                   typename ContextType::FeatureCloud::Ptr unmatches);
    void projectFeatureSpaces(typename ContextType::Ptr context,
                              typename ContextType::FeatureCloud::Ptr matches,
                              typename ContextType::FeatureCloud::Ptr unmatches);
  };
}

#define PCLREF_INSTANTIATE_LRTTransform(T,K,F) template class pclref::LRTTransform <T,K,F>;

#endif
