#include <EstimateNormals.h>
#include <DataTable.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/features/integral_image_normal.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  EstimateNormals<P, K, F>::EstimateNormals()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void EstimateNormals<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("en_time");
    names.push_back("en_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int EstimateNormals<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normalCloud(new typename ContextType::NormalCloud);

    double radius_search = this->m_params->template get<double>("normals_radius_search", 0.4f);
    bool remove_nans = this->m_params->template get<bool>("normals_remove_nans", true);
    bool filter_high_curvature = this->m_params->template get<bool>("normals_filter_high_curvature", false);

    bool parallel = this->m_params->template get<bool>("normals_parallel", true);
    bool integral = this->m_params->template get<bool>("use_integral_method", false);
    bool original_surface = this->m_params->template get<bool>("normals_original_surface", true);
    double curvature_threshold = this->m_params->template get<double>("normals_curvature_threshold", 0.03);

    if(integral)
    {
      pcl::IntegralImageNormalEstimation<P, pcl::Normal> ne;
      ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
      ne.setInputCloud(cloud);

      // params
      if(original_surface && context->exists("original_cloud"))
      {
        typename ContextType::PointCloud::Ptr surface = context->template get<typename ContextType::PointCloud::Ptr>("original_cloud");
        ne.setSearchSurface(surface);
      }
      ne.useSensorOriginAsViewPoint();
      ne.setMaxDepthChangeFactor(0.02f);
      ne.setNormalSmoothingSize(10.0f);
      ne.compute(*normalCloud);
    }
    else if(parallel)
    {
      pcl::NormalEstimationOMP<P, pcl::Normal> ne;
      ne.setInputCloud (cloud);

      // Params
      if(original_surface && context->exists("original_cloud"))
      {
        typename ContextType::PointCloud::Ptr surface = context->template get<typename ContextType::PointCloud::Ptr>("original_cloud");
        ne.setSearchSurface(surface);
      }
      ne.useSensorOriginAsViewPoint();
      ne.setRadiusSearch(radius_search);
      ne.compute (*normalCloud);
    }
    else
    {
      pcl::NormalEstimation<P, pcl::Normal> ne;
      ne.setInputCloud (cloud);

      // Params
      if(original_surface && context->exists("original_cloud"))
      {
        typename ContextType::PointCloud::Ptr surface = context->template get<typename ContextType::PointCloud::Ptr>("original_cloud");
        ne.setSearchSurface(surface);
      }
      ne.useSensorOriginAsViewPoint();
      ne.setRadiusSearch(radius_search);
      ne.compute (*normalCloud);
    }

    if(remove_nans)
    {
      std::vector<int> indices;
      typename ContextType::NormalCloud::Ptr normals_out(new typename ContextType::NormalCloud);
      pcl::removeNaNNormalsFromPointCloud (*normalCloud, *normals_out, indices); //NaNs
      if (indices.size () > 0)
      {
        typename ContextType::PointCloud::Ptr cloud2(new typename ContextType::PointCloud);
        copyPointCloud (*cloud, indices, *cloud2);
        if(context->exists("remapped_cloud")) context->set("remapped_cloud", cloud2);
        else context->set("cloud", cloud2);
        cloud = cloud2;
      }
      normalCloud = normals_out;
    }

    if(filter_high_curvature)
    {
      std::vector<int> indices;
      for(int idx = 0; idx < normalCloud->size(); ++idx)
      {
        if(normalCloud->points[idx].curvature <= curvature_threshold)
        {
          indices.push_back(idx);
        }
      }
      if (indices.size () > 0)
      {
        typename ContextType::PointCloud::Ptr cloud2(new typename ContextType::PointCloud);
        typename ContextType::NormalCloud::Ptr normals_out(new typename ContextType::NormalCloud);
        copyPointCloud (*cloud, indices, *cloud2);
        copyPointCloud (*normalCloud, indices, *normals_out);
        if(context->exists("remapped_cloud")) context->set("remapped_cloud", cloud2);
        else context->set("cloud", cloud2);
        cloud = cloud2;
        normalCloud = normals_out;
      }
    }

    context->set("normals", normalCloud);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, normalCloud->size());

    return normalCloud->size();
  }

  template<typename P, typename K, typename F>
  std::string EstimateNormals<P, K, F>::getName() const
  {
    return "EN";
  }
}

PCLREF_INSTANTIATE_PRODUCT(EstimateNormals,PCLREF_TYPES_PRODUCT)
