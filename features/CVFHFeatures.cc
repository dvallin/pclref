#include <CVFHFeatures.h>

#ifndef PCLREF_LEAN
#include <DataTable.h>

#include <pcl/features/cvfh.h>

namespace pclref
{
  template<typename P, typename K>
  CVFHFeatures<P, K>::CVFHFeatures()
  {
    init();
  }

  template<typename P, typename K>
  void CVFHFeatures<P, K>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("cvfh_time");
    names.push_back("cvfh_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K>
  int CVFHFeatures<P, K>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");
    typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);

    typename ContextType::IndexCloud::Ptr indices(new typename ContextType::IndexCloud);
    typename pcl::search::KdTree<P>::Ptr tree(new typename pcl::search::KdTree<P>);
    tree->setInputCloud(cloud);

    pcl::CVFHEstimation<P, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setClusterTolerance(this->m_params->template get<double>("cvfh_cluster_tolerance", 0.4));
    cvfh.setRadiusNormals(this->m_params->template get<double>("cvfh_normal_radius", 0.4));
    cvfh.setNormalizeBins(this->m_params->template get<bool>("cvfh_normalize_bins", false));
    cvfh.setCurvatureThreshold(this->m_params->template get<double>("cvfh_curv_threshold", 0.03));
    cvfh.setEPSAngleThreshold(this->m_params->template get<double>("cvfh_eps_angle_threshold", 0.125));
    cvfh.setMinPoints(this->m_params->template get<int>("cvfh_min_points", 50));

    // Compute the features
    cvfh.setInputNormals(normals);
    cvfh.setInputCloud(cloud);
    cvfh.compute(*features);

    std::vector<Eigen::Vector3f> centroids;
    cvfh.getCentroidClusters(centroids);
    BOOST_FOREACH(Eigen::Vector3f c, centroids)
    {
      std::vector<int> temp_ind(1);
      std::vector<float> temp_dist(1);
      P p;
      p.x = c[0];
      p.y = c[1];
      p.z = c[2];
      tree->nearestKSearch(p, 1, temp_ind, temp_dist);
      indices->push_back(temp_ind[0]);
    }
    context->set("keypoint_indices", indices);


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
  std::string CVFHFeatures<P, K>::getName() const
  {
    return "CVFH";
  }
}

PCLREF_INSTANTIATE_PRODUCT(CVFHFeatures,(PCLREF_POINT_TYPES)(PCLREF_KEYPOINT_TYPES))
#endif
