/**
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2012/07/24 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: May 22 2014
 *
 */

#ifndef PROCESSING_STEP_H_
#define PROCESSING_STEP_H_

#include <PclRefLIB.h>
#include <DataTable.h>
#include <ProcessingContext.h>
#include <ParameterContext.h>

namespace pclref
{
  /**
   * \brief The abstract class every algorithm in this framework is derived from.
   *
   * The ProcessingStep provides two logging systems. The main logger used to write
   * information such as execution times or quality metrics and a set of additional logs
   * in which histograms or matrices can be written.
   *
   * When calling compute a ProcessingContext must be given. Each ProcessingStep
   * fetches named objects from this context and writes results to it.
   * The actual names and types used are documented as preconditions and postconditions.
   *
   * Parameters are set for each ProcessingStep using a ParameterContext. Name, type and
   * standard value for each parameter are documented along the specific class.
   *
   * A good example for this documentation practice is the documentation of pclref::Harris3DKeypoints.
   */
  template <typename PointType, typename KeypointType, typename FeatureType>
  class ProcessingStep
  {
  public:
    typedef boost::shared_ptr<ProcessingStep<PointType, KeypointType, FeatureType> > Ptr; ///< type of smart pointer of this class
    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    ProcessingStep()
    {
    }

    /*! Do all your initialization here.
     *  \note parameters are not set at this point.
     *  \note you need to create a new logger here. Example:
     *  \code{.cpp}
     *  std::vector<DataTable::columnType> types;
     *  types.push_back(DataTable::FLOAT);
     *  types.push_back(DataTable::FLOAT);
     *  std::vector<std::string> names;
     *  names.push_back("narff_time");
     *  names.push_back("narff_size");
     *  this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
     *  \endcode
     */
    virtual void init()
    {
      m_params.reset();
    }

    /// do your initialization of the additional logs here. Parameters are already set now.
    virtual void prepareAdditionalLogs()
    {
    }

    /** Do all your computations here
     * \param run the number of the current execution, this is crucial for parallel execution
     * \param context the context to be used
     * \note see documentation of each ProcessingStep you use for
     * preconditions of ProcessingContext and available parameters of
     * ParameterContext
     * \return a success result, 0 is a failure, any other value is considered a success!
     */
    virtual int compute(size_t run,
                        typename ContextType::Ptr context) = 0;

    /** Returns a short name of this algorithm. It is primarily used to
      * create the name of the pipeline, so you should be brief.
     * \return name of the current ProcessingStep
      */
    virtual std::string getName() const = 0;

    /** Sets the ParameterContext to be used by this step.
     * \note you must set it before calling Compute, but may reset it any time
     * \param context the ParameterContext that should be used.
     */
    void setParameterContext(typename ParameterContext::Ptr context)
    {
      m_params = context;
    }

    /** returns the logger of this ProcessingStep, containing all relevant information about performance and whatnot.
    * \return the log
    */
    typename DataAccessor::Ptr getLogger() const
    {
      return m_logger;
    }

    /** returns the additional logs
    * \return array to the logs
    */
    const std::vector<typename DataAccessor::Ptr>& getAdditionalLogs() const
    {
      return m_additional_logs;
    }

  protected:
    ParameterContext::Ptr m_params; ///< the ParameterContext
    typename DataAccessor::Ptr m_logger; ///< the Logger
    std::vector<typename DataAccessor::Ptr> m_additional_logs; ///< the Array of additional Logs
  };
}

#endif
