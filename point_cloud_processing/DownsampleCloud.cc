#include <DownsampleCloud.h>
#include <DataTable.h>

#include <pcl/filters/voxel_grid.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  DownsampleCloud<P, K, F>::DownsampleCloud()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void DownsampleCloud<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ds_time");
    names.push_back("ds_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int DownsampleCloud<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    float leaf_size = this->m_params->template get<double>("model_resolution", 0.05);

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("cloud");
    typename ContextType::PointCloud::Ptr filtered_cloud(new typename ContextType::PointCloud);

    pcl::VoxelGrid<P> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leaf_size, leaf_size, leaf_size);
    sor.filter (*filtered_cloud);

    context->set("original_cloud", cloud);
    context->set("cloud", filtered_cloud);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, filtered_cloud->size());

    return filtered_cloud->size();
  }

  template<typename P, typename K, typename F>
  std::string DownsampleCloud<P, K, F>::getName() const
  {
    return "DS";
  }
}

PCLREF_INSTANTIATE_PRODUCT(DownsampleCloud,PCLREF_TYPES_PRODUCT)
