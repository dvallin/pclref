#include <ISSKeypoints.h>
#include <DataTable.h>

#include <pcl/keypoints/iss_3d.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  ISSKeypoints<P, K, F>::ISSKeypoints()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void ISSKeypoints<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("iss_time");
    names.push_back("iss_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int ISSKeypoints<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    typedef pcl::ISSKeypoint3D<P, K>  ISSType;

#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");
    typename ContextType::KeypointCloud::Ptr keypoints(new typename ContextType::KeypointCloud);

    //float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);
    double iss_salient_radius = this->m_params->template get<double>("iss_salient_radius", 0.6); //6*model_resolution);
    double iss_non_max_radius =  this->m_params->template get<double>("iss_non_max_radius", 0.4); //4*model_resolution);
    double iss_gamma_21 = this->m_params->template get<double>("iss_gamma_21", 0.975);
    double iss_gamma_32 = this->m_params->template get<double>("iss_gamma_32", 0.975);
    int iss_min_neighbors = this->m_params->template get<int>("iss_min_neighbors", 5);
    int iss_threads = this->m_params->template get<int>("iss_threads", 4);

    bool use_border_detection = this->m_params->template get<bool>("iss_border_detection", false);

    //
    // Compute keypoints
    //
    ISSType iss_detector;

    if(use_border_detection)
    {
      iss_detector.setBorderRadius (this->m_params->template get<double>("iss_border_radius", 0.1));
      iss_detector.setNormalRadius (this->m_params->template get<double>("iss_normal_radius", 0.4));
    }
    iss_detector.setSalientRadius (iss_salient_radius);
    iss_detector.setNonMaxRadius (iss_non_max_radius);
    iss_detector.setThreshold21 (iss_gamma_21);
    iss_detector.setThreshold32 (iss_gamma_32);
    iss_detector.setMinNeighbors (iss_min_neighbors);
    iss_detector.setNumberOfThreads (iss_threads);
    iss_detector.setInputCloud (cloud);
    iss_detector.setNormals(normals);
    iss_detector.compute (*keypoints);

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
  std::string ISSKeypoints<P, K, F>:: getName() const
  {
    return "ISS";
  }
}

PCLREF_INSTANTIATE_PRODUCT(ISSKeypoints,((pcl::PointXYZ))((pcl::PointXYZ))(PCLREF_FEATURE_TYPES))
