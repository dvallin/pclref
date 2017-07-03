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
 * Created on: June 4 2014
 *
 */

#ifndef COMBINER_H_
#define COMBINER_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>

namespace pclref
{
  /**
   * \class Combiner
   *
   * \brief Combines multiple Processing Steps in one Pipeline
   *
   * This class expects a ProcessingContext of ProcessingContexts. Each is
   * used for one execution of the pipeline. Add Pipelinesteps in order of
   * their execution.
   *
   * The example application in main.cc shows how this could be done using the registerRuns function
  * \code
   * typename ContextType::Ptr kfContexts(new ContextType);
   * typename ContextType::Ptr ceContexts(new ContextType);
   * typename ContextType::Ptr accuContexts(new ContextType);
  *
  *   registerRuns(meta, kfContexts, params, runs);
  * \endcode
  *
  * To create a set of contexts containing point cloud pairs (and additionally fullfilling some
  * common preconditions for typical point cloud pair steps) you could proceed as following:
  *
  * \code
  * int k = 0;
  *    for(iterI: all contexts in kfContexts)
  *    {
  *      for(iterJ: all contexts in kfContexts)
  *      {
  *        if(iterJ == iterI)
  *          continue;
  *
  *          typename ContextType::Ptr context(new ContextType);
  *
  *          typename ContextType::Ptr sourceContext = boost::any_cast<typename ContextType::Ptr>(*iterJ);
  *          typename ContextType::Ptr targetContext = boost::any_cast<typename ContextType::Ptr>(*iterI);
  *
  *          context->add("source_id", sourceContext->template get<identifier>("id"));
  *          context->add("target_id", targetContext->template get<identifier>("id"));
  *
  *          context->add("cloud_source", sourceContext->template get<typename ContextType::PointCloud::Ptr>("cloud"));
  *          context->add("cloud_target", targetContext->template get<typename ContextType::PointCloud::Ptr>("cloud"));
  *          [...]
  *          ceContexts->add("context_" + precision_cast(k++, 1), context);
  *        }
  *      }
  * \endcode
  *
  *
   * Some pipeline steps can be run in an accumulated mode. This means that
   * they get the results of all the previous pipeline runs:
  * \code
  *  typename ContextType::Ptr accu(new ContextType);
  *      accu->add("kfContexts", kfContexts);
  *      accu->add("ceContexts", ceContexts);
  *      accu->add("accu", true);
  *      accuContexts->add("context", accu);
  * \endcode
   *
   *
   */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class Combiner : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    Combiner();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

    void addStep(Ptr step);

  protected:
    boost::shared_ptr<DataAccessor> m_logger_internal;
    std::vector<Ptr> m_steps;
  };

}

#define PCLREF_INSTANTIATE_Combiner(T,K,F) template class pclref::Combiner <T,K,F>;

#endif
