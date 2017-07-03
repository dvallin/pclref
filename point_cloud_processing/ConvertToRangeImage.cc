#include <ConvertToRangeImage.h>
#include <DataTable.h>

#include <pcl/io/pcd_io.h>

namespace pclref
{

  template<typename P, typename K, typename F>
  ConvertToRangeImage<P, K, F>::ConvertToRangeImage()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void ConvertToRangeImage<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ri_time");
    names.push_back("ri_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int ConvertToRangeImage<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("cloud");
    typename ContextType::RangeImage::Ptr range_image(new typename ContextType::RangeImage);

    // think about these!
    float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);
    float angular_resolution_x = 2;//sqrtf(model_resolution);
    float angular_resolution_y = 1;//sqrtf(model_resolution);
    typename ContextType::RangeImage::CoordinateFrame coordinate_frame = ContextType::RangeImage::CAMERA_FRAME;
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
    angular_resolution_x = pcl::deg2rad (angular_resolution_x);
    angular_resolution_y = pcl::deg2rad (angular_resolution_y);

    range_image->createFromPointCloud (*cloud, angular_resolution_x, angular_resolution_y,
                                       pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                       scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

    typename ContextType::PointCloud::Ptr filtered_cloud(new typename ContextType::PointCloud);
    for(int i = 0; i < range_image->size(); ++i)
    {
      pcl::PointWithRange p = range_image->getPoint(i);
      P p2;
      p2.x = p.x;
      p2.y = p.y;
      p2.z = p.z;
      filtered_cloud->points.push_back(p2);
    }


    context->set("range_image", range_image);
    context->set("cloud_original", cloud);
    context->set("cloud", filtered_cloud);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, range_image->size());

    return filtered_cloud->size();
  }

  template<typename P, typename K, typename F>
  std::string ConvertToRangeImage<P, K, F>::getName() const
  {
    return "RI";
  }
}


PCLREF_INSTANTIATE_PRODUCT(ConvertToRangeImage,PCLREF_TYPES_PRODUCT)
