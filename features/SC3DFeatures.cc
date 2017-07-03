#include <SC3DFeatures.h>

#ifndef PCLREF_LEAN
#include <DataTable.h>

#include <pcl/features/3dsc.h>

namespace pclref
{

  template<typename P, typename K>
  SC3DFeatures<P, K>::SC3DFeatures()
  {
    init();
  }

  template<typename P, typename K>
  void SC3DFeatures<P, K>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("sc3d_time");
    names.push_back("sc3d_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K>
  int SC3DFeatures<P, K>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");
    typename ContextType::IndexCloud::Ptr keypoint_indices = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices");
    typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);

    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
    indices->reserve(keypoint_indices->size());
    BOOST_FOREACH(int index, *keypoint_indices)
    {
      indices->push_back(index);
    }

    pcl::ShapeContext3DEstimation<P, pcl::Normal, pcl::ShapeContext1980> sc3d;
    sc3d.setRadiusSearch(this->m_params->template get<double>("sc3d_radius_search", 0.4));
    sc3d.setMinimalRadius(this->m_params->template get<double>("sc3d_minimal_radius", 0.2));
    sc3d.setPointDensityRadius(this->m_params->template get<double>("sc3d_point_density_radius", 0.2));

    // Compute the features
    sc3d.setIndices(indices);
    sc3d.setInputNormals(normals);
    sc3d.setInputCloud(cloud);
    sc3d.compute(*features);

    // copy cloud origin/orientation
    features->sensor_orientation_ = cloud->sensor_orientation_;
    features->sensor_origin_ = cloud->sensor_origin_;

    context->set("features", features);

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
  std::string SC3DFeatures<P, K>::getName() const
  {
    return "SC3D";
  }
}

PCLREF_INSTANTIATE_PRODUCT(SC3DFeatures,(PCLREF_POINT_TYPES)(PCLREF_KEYPOINT_TYPES))
#endif
