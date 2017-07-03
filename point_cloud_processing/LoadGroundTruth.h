/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2014/07/31 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: July 31 2014
 *
 */

#ifndef LOAD_GROUND_TRUTH_H_
#define LOAD_GROUND_TRUTH_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class LoadGroundTruth
  *
  * \brief Loads ground truth transformation from a pre-parsed csv-like file
  *
  * \pre DataAccessor gt_structure : pre-parsed csv-file containing ground truth
  * \pre int gt_col : column of DataAccessor to start parsing
  * \pre int gt_line : row of DataAccessor to start parsing
  * \pre bool gt_inline : bool true for inline parsing, else rectangular
  * \pre bool gt_matrix : bool true for matrix parsing, else translation+euler angles
  *
  * \post ProcessingContext::Transformation grount_truth
  *
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class LoadGroundTruth : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    LoadGroundTruth();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;
  };
}

#define PCLREF_INSTANTIATE_LoadGroundTruth(T,K,F) template class pclref::LoadGroundTruth <T,K,F>;

#endif
