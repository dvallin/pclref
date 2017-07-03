#include <VFHFeatures.h>

#ifndef PCLREF_LEAN
#include <DataTable.h>

#include <pcl/features/vfh.h>

namespace pclref
{
  template<typename P, typename K>
  VFHFeatures<P, K>::VFHFeatures()
  {
    init();
  }

  template<typename P, typename K>
  void VFHFeatures<P, K>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("vfh_time");
    names.push_back("vfh_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K>
  int VFHFeatures<P, K>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");
    boost::shared_ptr<typename ContextType::Clusters> clusters = context->template get<boost::shared_ptr<typename ContextType::Clusters> >("clusters");
    typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);


    pcl::VFHEstimation<P, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (normals);
    vfh.setUseGivenNormal (true);
    vfh.setUseGivenCentroid (true);
    vfh.setNormalizeBins (this->m_params->template get<bool>("vfh_normalize_bins", true));
    vfh.setNormalizeDistance (this->m_params->template get<bool>("vfh_normalize_distance", false));
    vfh.setFillSizeComponent (this->m_params->template get<bool>("vfh_fill_size_component", true));

    // Compute the features
    for(size_t idx = 0 ; idx < clusters->size(); ++idx)
    {
      typename ContextType::FeatureCloud f;
      boost::shared_ptr<std::vector<int> > indices(& ((*clusters)[idx].indices), null_deleter());

      Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero ();
      Eigen::Vector3f centroid = Eigen::Vector3f::Zero ();
      BOOST_FOREACH(int i, *indices)
      {
        avg_normal += normals->points[i].getNormalVector3fMap ();
        centroid += cloud->points[i].getVector3fMap ();
      }
      avg_normal /= static_cast<float> (indices->size ());
      centroid /= static_cast<float> (indices->size ());
      avg_normal.normalize ();

      //configure VFH computation for CVFH
      vfh.setNormalToUse (avg_normal);
      vfh.setCentroidToUse (centroid);
      vfh.setIndices(indices);
      vfh.compute(f);
      features->points.push_back(f.points.back());
    }

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
  std::string VFHFeatures<P, K>::getName() const
  {
    return "VFH";
  }
}

PCLREF_INSTANTIATE_PRODUCT(VFHFeatures,(PCLREF_POINT_TYPES)(PCLREF_KEYPOINT_TYPES))
#endif
