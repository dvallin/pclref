#include <Uniqueness.h>
#include <DataTable.h>
#include <Histogram.h>

#include <pcl/common/transforms.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  Uniqueness<P, K, F>::Uniqueness()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void Uniqueness<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("uq_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void Uniqueness<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();
    bool write_histos = this->m_params->template get<bool>("uniqueness_write_histos", false);
    if(write_histos)
    {
      float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);
      int uniqueness_k = this->m_params->template get<int>("uniqueness_k", 5);

      for(int i = 0; i < uniqueness_k; ++i)
      {
        typename Histogram::Ptr hist(new Histogram(0.0, 6*model_resolution, 60, false));
        hist->setName("nn" + precision_cast(i, 1) + "-histogram");
        this->m_additional_logs.push_back(hist);
      }
    }
  }

  template<typename P, typename K, typename F>
  int Uniqueness<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    bool write_histos = this->m_params->template get<bool>("uniqueness_write_histos", false);

    if(write_histos)
    {
      int uniqueness_k = this->m_params->template get<int>("uniqueness_k", 5);
      assert(this->m_additional_logs.size() == uniqueness_k);

      typename ContextType::KeypointCloud::Ptr keypoints_target = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_target");
      typename ContextType::PointCloud::Ptr cloud_source = context->template get<typename ContextType::PointCloud::Ptr>("cloud_source");
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_source = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_target = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
      typename ContextType::PointCloud::Ptr cloud_trans_source(new typename ContextType::PointCloud);
      typename ContextType::KeypointCloud::Ptr keypoints_trans_target(new typename ContextType::KeypointCloud);
      pcl::transformPointCloud(*cloud_source, *cloud_trans_source, *ground_truth_source);
      pcl::transformPointCloud(*keypoints_target, *keypoints_trans_target, *ground_truth_target);

      typename pcl::search::KdTree<K>::Ptr keypoint_search(new typename pcl::search::KdTree<K>);
      keypoint_search->setInputCloud(keypoints_trans_target);

      typename ContextType::IndexCloud::Ptr keypoints_overlap = context->template get<typename ContextType::IndexCloud::Ptr>("keypoints_overlap");

      size_t c = keypoints_overlap->size();

      for (size_t i = 0; i < c; ++i)
      {
        int idx = keypoints_overlap->points[i];
        P p = cloud_trans_source->points[idx];

        K k;
        k.x = p.x;
        k.y = p.y;
        k.z = p.z;

        std::vector<int> idx_target;
        std::vector<float> dst;
        keypoint_search->nearestKSearch(k, uniqueness_k, idx_target, dst);

        for(int j = 0; j < uniqueness_k; ++j)
        {
          typename Histogram::Ptr hist = boost::dynamic_pointer_cast<Histogram>
                                         (this->m_additional_logs[j]);
          hist->add(dst[j], run);
        }
      }
    }

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string Uniqueness<P, K, F>::getName() const
  {
    return "UQ";
  }
}

PCLREF_INSTANTIATE_PRODUCT(Uniqueness,PCLREF_TYPES_PRODUCT)
