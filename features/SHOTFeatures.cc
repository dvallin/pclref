#include <SHOTFeatures.h>

#include <DataTable.h>

#include <pcl/features/shot.h>

namespace pclref
{
  template<typename P, typename K>
  SHOTFeatures<P, K>::SHOTFeatures()
  {
    init();
  }

  template<typename P, typename K>
  void SHOTFeatures<P, K>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("shot_time");
    names.push_back("shot_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K>
  int SHOTFeatures<P, K>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");
    typename ContextType::IndexCloud::Ptr keypoint_indices = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices");
    typename ContextType::KeypointCloud::Ptr keypoints = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints");
    typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);
    typename ContextType::FeatureCloud::Ptr features_filtered(new typename ContextType::FeatureCloud);
    typename ContextType::IndexCloud::Ptr keypoint_indices_filtered(new typename ContextType::IndexCloud);
    typename ContextType::KeypointCloud::Ptr keypoints_filtered(new typename ContextType::KeypointCloud);

    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
    indices->reserve(keypoint_indices->size());
    BOOST_FOREACH(int index, *keypoint_indices)
    {
      indices->push_back(index);
    }

    pcl::SHOTEstimation<P, pcl::Normal, pcl::SHOT352> shot;
    shot.setRadiusSearch(this->m_params->template get<double>("shot_radius_search", 1.0));
    shot.setLRFRadius(this->m_params->template get<double>("shot_lrf_radius", 0.8));

    // Compute the features
    shot.setInputNormals(normals);
    shot.setIndices(indices);
    shot.setInputCloud(cloud);
    shot.compute(*features);
    for(size_t idx = 0; idx < features->size(); ++idx)
    {
      pcl::SHOT352 f = features->points[idx];
      if(isValid(f))
      {
        K k = keypoints->points[idx];
        int i = keypoint_indices->points[idx];
        features_filtered->points.push_back(f);
        keypoints_filtered->points.push_back(k);
        keypoint_indices_filtered->points.push_back(i);
      }
    }

    // copy cloud origin/orientation
    features_filtered->sensor_orientation_ = cloud->sensor_orientation_;
    features_filtered->sensor_origin_ = cloud->sensor_origin_;

    context->set("features", features_filtered);
    context->set("keypoints", keypoints_filtered);
    context->set("keypoint_indices", keypoint_indices_filtered);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, features->size());

    return features->size();
  }

  template<typename P, typename K>
  std::string SHOTFeatures<P, K>::getName() const
  {
    return "SHOT";
  }
}

PCLREF_INSTANTIATE_PRODUCT(SHOTFeatures,(PCLREF_POINT_TYPES)(PCLREF_KEYPOINT_TYPES))

