#include <USCFeatures.h>

#ifndef PCLREF_LEAN
#include <DataTable.h>

#include <pcl/features/usc.h>
#include <pcl/impl/point_types.hpp>


namespace pclref
{

  template<typename P, typename K>
  USCFeatures<P, K>::USCFeatures()
  {
    init();
  }

  template<typename P, typename K>
  void USCFeatures<P, K>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("usc_time");
    names.push_back("usc_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K>
  int USCFeatures<P, K>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::IndexCloud::Ptr keypoint_indices = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices");
    typename ContextType::KeypointCloud::Ptr keypoints = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints");
    typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);

    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
    indices->reserve(keypoint_indices->size());
    BOOST_FOREACH(int index, *keypoint_indices)
    {
      indices->push_back(index);
    }

    pcl::UniqueShapeContext<P, pcl::ShapeContext1980> usc;
    usc.setRadiusSearch(this->m_params->template get<double>("usc_radius_search", 0.4));
    usc.setLocalRadius(this->m_params->template get<double>("usc_local_radius", 0.2));
    usc.setMinimalRadius(this->m_params->template get<double>("usc_minimal_radius", 0.1));
    usc.setPointDensityRadius(this->m_params->template get<double>("usc_point_density_radius", 0.2));

    // Compute the features
    usc.setIndices(indices);
    usc.setInputCloud(cloud);
    usc.compute(*features);

    std::vector<int> in;
    for(int i = 0; i < features->size(); ++i)
    {
      if(pclref::isValid(features->points[i])) in.push_back(i);
    }

    typename ContextType::IndexCloud::Ptr keypoint_indices_in(new typename ContextType::IndexCloud);
    typename ContextType::KeypointCloud::Ptr keypoints_in(new typename ContextType::KeypointCloud);
    typename ContextType::FeatureCloud::Ptr features_in(new typename ContextType::FeatureCloud);
    for(int i = 0; i < in.size(); ++i)
    {
      keypoint_indices_in->points.push_back(keypoint_indices->points[in[i]]);
      keypoints_in->points.push_back(keypoints->points[in[i]]);
      features_in->points.push_back(features->points[in[i]]);
    }

    // copy cloud origin/orientation
    features->sensor_orientation_ = cloud->sensor_orientation_;
    features->sensor_origin_ = cloud->sensor_origin_;

    context->set("keypoints", keypoints_in);
    context->set("keypoint_indices", keypoint_indices_in);
    context->set("features", features_in);

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
  std::string USCFeatures<P, K>::getName() const
  {
    return "USC";
  }
}

PCLREF_INSTANTIATE_PRODUCT(USCFeatures,(PCLREF_POINT_TYPES)(PCLREF_KEYPOINT_TYPES))
#endif
