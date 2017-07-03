#include <PointCloudStats.h>
#include <DataTable.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  PointCloudStats<P, K, F>::PointCloudStats()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void PointCloudStats<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ps_time");
    names.push_back("ps_min_x");
    names.push_back("ps_max_x");
    names.push_back("ps_min_y");
    names.push_back("ps_max_y");
    names.push_back("ps_min_z");
    names.push_back("ps_max_z");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int PointCloudStats<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");

    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*cloud, min, max);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, min[0]);
    this->m_logger->set(run, 2, max[0]);
    this->m_logger->set(run, 3, min[1]);
    this->m_logger->set(run, 4, max[1]);
    this->m_logger->set(run, 5, min[2]);
    this->m_logger->set(run, 6, max[2]);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string PointCloudStats<P, K, F>::getName() const
  {
    return "PS";
  }
}

PCLREF_INSTANTIATE_PRODUCT(PointCloudStats,PCLREF_TYPES_PRODUCT)
