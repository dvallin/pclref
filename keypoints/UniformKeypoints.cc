#include <UniformKeypoints.h>
#include <DataTable.h>

#include <pcl/keypoints/uniform_sampling.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  UniformKeypoints<P, K, F>::UniformKeypoints()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void UniformKeypoints<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("uk_time");
    names.push_back("uk_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int UniformKeypoints<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    typedef pcl::UniformSampling<P> UKType;

#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::IndexCloud::Ptr keypoints(new typename ContextType::IndexCloud);

    typename UKType::Ptr uk(new UKType);
    uk->setInputCloud(cloud);
    uk->setRadiusSearch(this->m_params->template get<double>("uk_radius", 0.4));
    uk->compute(*keypoints);

    keypoints->sensor_orientation_ = cloud->sensor_orientation_;
    keypoints->sensor_origin_ = cloud->sensor_origin_;

    context->set("keypoint_indices", keypoints);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, keypoints->size());

    return keypoints->size();
  }

  template<typename P, typename K, typename F>
  std::string UniformKeypoints<P, K, F>::getName() const
  {
    return "UK";
  }
}

PCLREF_INSTANTIATE_PRODUCT(UniformKeypoints,PCLREF_TYPES_PRODUCT)
