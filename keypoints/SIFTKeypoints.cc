#include <SIFTKeypoints.h>

#ifndef PCLREF_LEAN
#include <DataTable.h>

#include <pcl/keypoints/sift_keypoint.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  SIFTKeypoints<P, K, F>::SIFTKeypoints()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void SIFTKeypoints<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("sift_time");
    names.push_back("sift_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int SIFTKeypoints<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    typedef pcl::SIFTKeypoint<pcl::PointXYZI, K> SIFTType;

#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");
    typename ContextType::KeypointCloud::Ptr keypoints(new typename ContextType::KeypointCloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_normals->points.resize(normals->points.size());
    for(size_t i = 0; i < cloud->points.size(); ++i)
    {
      cloud_normals->points[i].x = cloud->points[i].x;
      cloud_normals->points[i].y = cloud->points[i].y;
      cloud_normals->points[i].z = cloud->points[i].z;
    }

    if(this->m_params->template get<bool>("sift_use_curvature", true))
    {
      for(size_t i = 0; i < cloud->points.size(); ++i)
      {
        cloud_normals->points[i].intensity = normals->points[i].curvature;
      }
    }
    else
    {
      for(size_t i = 0; i < cloud->points.size(); ++i)
      {
        float dx = cloud->points[i].x - cloud->sensor_origin_[0];
        float dy = cloud->points[i].y - cloud->sensor_origin_[1];
        float dz = cloud->points[i].z - cloud->sensor_origin_[2];
        cloud_normals->points[i].intensity = dx*dx + dy*dy + dz*dz;
      }
    }

    typename SIFTType::Ptr sift(new SIFTType);
    sift->setMinimumContrast(this->m_params->template get<double>("sift_min_contrast", 0.001));
    sift->setScales (this->m_params->template get<double>("sift_min_scale", 0.1),
                     this->m_params->template get<int>("sift_octaves", 3),
                     this->m_params->template get<int>("sift_scales_per_octave", 4));
    sift->setInputCloud(cloud_normals);
    sift->compute(*keypoints);

    keypoints->sensor_orientation_ = cloud->sensor_orientation_;
    keypoints->sensor_origin_ = cloud->sensor_origin_;

    context->set("keypoints", keypoints);

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
  std::string SIFTKeypoints<P, K, F>::getName() const
  {
    return "SIFT";
  }
}

PCLREF_INSTANTIATE_PRODUCT(SIFTKeypoints,((pcl::PointXYZ))((pcl::PointWithScale))(PCLREF_FEATURE_TYPES))
#endif
