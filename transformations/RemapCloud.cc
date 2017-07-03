#include <RemapCloud.h>
#include <DataTable.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  RemapCloud<P, K, F>::RemapCloud()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void RemapCloud<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("rc_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int RemapCloud<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr> ("cloud");
    typename ContextType::PointCloud::Ptr cloud_r(new typename ContextType::PointCloud);
    float exp = this->m_params->template get<double>("remap_exponent", 1.0);
    float scale = this->m_params->template get<double>("remap_scale", 1.0);
    float cutoff = this->m_params->template get<double>("remap_cutoff", std::numeric_limits<double>::max());

    Eigen::Vector4f origin = cloud->sensor_origin_;
    for(size_t idx = 0; idx < cloud->points.size(); ++idx)
    {
      P p = cloud->points[idx];
      Eigen::Vector4f v = XYZtoEigenVector4<P, float>(p);
      Eigen::Vector4f d = v - origin;
      float length = d.norm();
      d /= length;
      if(length < cutoff)
      {
        float t = scale*(std::pow(length,exp));
        //float t = scale*atan(length*exp);
        Eigen::Vector4f p_r = origin + (t*d);
        cloud_r->points.push_back(EigenVector4toXYZ<P>(p_r));
      }
    }

    cloud_r->sensor_origin_ = cloud->sensor_origin_;
    cloud_r->sensor_orientation_ = cloud->sensor_orientation_;

    context->set("remapped_cloud", cloud_r);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string RemapCloud<P, K, F>::getName() const
  {
    return "RC";
  }
}

PCLREF_INSTANTIATE_PRODUCT(RemapCloud,PCLREF_TYPES_PRODUCT)
