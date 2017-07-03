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

#ifndef CDF_TRANSFORM_H_
#define CDF_TRANSFORM_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class CDFTransform
  *
  * \brief Transforms feature spaces using a inverse cumulative distribution function transform.
  *
  * \param cdf_write : bool (false)
  *
  * There are two use cases for this class. Either a cdf_stats structure is present in which case
  * a transformation should be applied to the current feature cloud. Or a transformation should be
  * estimated based on some feature clouds and written to the additional logs.
  *
  * For writing the trained matrix cdf_write must be set.
  *
  * To train a transformation
  * \pre bool accu
  * \pre ProcessingContext kfContexts
  *
  * each context in kfContexts:
  * \pre ProcessingContext::FeatureCloud features
  * \post Eigen::MatrixXf cdf_stats
  *
  * To project features:
  * \pre DataAccessor cdf_stats
  * \pre ProcessingContext::FeatureCloud features
  * \post ProcessingContext::FeatureCloud features
  *
  * \see pclref::CDF
  *
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class CDFTransform : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    CDFTransform();

    void init();
    void prepareAdditionalLogs();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

  private:
    void generateInputFeatureSpace(typename ContextType::Ptr context, typename ContextType::FeatureCloud::Ptr features);
    void projectFeatureSpaces(typename ContextType::Ptr context, typename ContextType::FeatureCloud::Ptr features);
  };
}

#define PCLREF_INSTANTIATE_CDFTransform(T,K,F) template class pclref::CDFTransform <T,K,F>;

#endif
