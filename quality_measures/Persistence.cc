#include <Persistence.h>
#include <DataTable.h>
#include <ProcessingStep.h>
#include <Histogram.h>
#include <Arrays.h>
#include <MersenneTwister.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  Persistence<P, K, F>::Persistence()
  {
#ifdef PCLREF_PARALLEL
    omp_init_lock(&m_writelock);
#endif
    init();
  }

  template<typename P, typename K, typename F>
  Persistence<P, K, F>::~Persistence()
  {
#ifdef PCLREF_PARALLEL
    omp_destroy_lock(&m_writelock);
#endif
  }

  template<typename P, typename K, typename F>
  void Persistence<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("pe_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void Persistence<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();

    bool write_distances = this->m_params->template get<bool>("pe_write_distances", false);
    if(write_distances)
    {
      typename Histogram::Ptr hist(new Histogram(0.0, 100.0, 60));
      hist->setName("persistence-histogram");
      this->m_additional_logs.push_back(hist);
    }
  }

  template<typename P, typename K, typename F>
  int Persistence<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif
    typename ContextType::KeypointCloud::Ptr keypoints = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints");
    typename ContextType::IndexCloud::Ptr keypoint_indices = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices");

    typename ContextType::FeatureCloud::Ptr features_out(new typename ContextType::FeatureCloud);
    typename ContextType::FeatureCloud::Ptr features = context->template get<typename ContextType::FeatureCloud::Ptr>("features");
    typename ContextType::KeypointCloud::Ptr keypoints_out(new typename ContextType::KeypointCloud);
    typename ContextType::IndexCloud::Ptr keypoint_indices_out(new typename ContextType::IndexCloud);

    analyze(features, features_out, keypoints, keypoints_out, keypoint_indices, keypoint_indices_out, run);

    context->set("features", features_out);
    context->set("keypoints", keypoints_out);
    context->set("keypoint_indices", keypoint_indices_out);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string Persistence<P, K, F>::getName() const
  {
    return "PE";
  }

  template<typename P, typename K, typename F>
  template<typename T>
  void Persistence<P, K, F>::analyze(boost::shared_ptr<pcl::PointCloud<T> >& features, boost::shared_ptr<pcl::PointCloud<T> >& features_out,
                                     typename ContextType::KeypointCloud::Ptr& keypoints, typename ContextType::KeypointCloud::Ptr& keypoints_out,
                                     typename ContextType::IndexCloud::Ptr& keypoint_indices, typename ContextType::IndexCloud::Ptr& keypoint_indices_out,
                                     size_t run)
  {
    if(features->size() > 0)
    {
      Eigen::VectorXd centroid;
      std::vector<int> indices;
      Arrays::natural(0, (int)features->size(), indices);
      pclref::computeCentroid(*features, indices, centroid);
      T mean_feature = pclref::fromEigenVector<T>(centroid);

      double max = 0.0;
      double sd = 0.0;
      for(size_t i = 0; i < features->size(); ++i)
      {
        double dist = distance(mean_feature, features->points[i], pcl::L2);
        max = std::max(max, dist);
        sd += dist*dist;
      }
      sd = std::sqrt(sd / features->size());

      float min_theta = sd * this->m_params->template get<double>("pe_min_sigma", 0.0);
      float max_theta = sd * this->m_params->template get<double>("pe_max_sigma", std::numeric_limits<double>::max());
      for(size_t i = 0; i < features->size(); ++i)
      {
        double dist = distance(mean_feature, features->points[i], pcl::L2);
        if(dist > min_theta && dist < max_theta)
        {
          features_out->push_back(features->points[i]);
          keypoints_out->push_back(keypoints->points[i]);
          keypoint_indices_out->push_back(keypoint_indices->points[i]);
        }
      }
      bool write_distances = this->m_params->template get<bool>("pe_write_distances", false);
      if(write_distances)
      {
        typename Histogram::Ptr hist = boost::dynamic_pointer_cast<Histogram>(this->m_additional_logs[0]);
        hist->setRange(0.0, max);
        for(size_t i = 0; i < features_out->size(); ++i)
        {
          double dist = distance(mean_feature, features_out->points[i], pcl::L2);
          hist->add(dist, run);
        }
      }
    }
  }
}

PCLREF_INSTANTIATE_PRODUCT(Persistence,PCLREF_TYPES_PRODUCT)
