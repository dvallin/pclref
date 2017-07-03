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
 * Created on: May 22 2014
 *
 */

#ifndef LOAD_CLOUD_H_
#define LOAD_CLOUD_H_

#include <ProcessingStep.h>

namespace pclref
{
  /**
  * \class LoadCloud
  *
  * \brief loads a PointCloud from either a .pcd file or a DataAccessor
  *
  * This class automatically detects the type of load_cloud_structure. If it
  * is a std::string it assumes, that it can directly be loaded by the PCL.
  * If it is a DataAccessor it interprets the columns given by the parameters
  * Coordinates. Either way it constructs a PointCloud.
  *
  * \param x_column : int (0) column of DataAccessor containing x values
  * \param y_column : int (1) column of DataAccessor containing y values
  * \param z_column : int (2) column of DataAccessor containing z values
  *
  * \pre either std::string or DataAccessor load_cloud_structure
  * \post ProcessingContext::PointCloud cloud
  *
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class LoadCloud : public ProcessingStep<PointType, KeypointType, FeatureType>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, FeatureType> BaseType; ///< The type of the base class
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    LoadCloud();

    void init();

    int compute(size_t run, typename ContextType::Ptr context);

    std::string getName() const;

  private:
    void process(const std::string& filename, typename ContextType::PointCloud::Ptr cloud);
    void process(const DataAccessor::Ptr& acc, typename ContextType::PointCloud::Ptr cloud);
  };
}

#define PCLREF_INSTANTIATE_LoadCloud(T,K,F) template class pclref::LoadCloud <T,K,F>;

#endif
