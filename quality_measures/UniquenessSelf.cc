#include <UniquenessSelf.h>
#include <DataTable.h>
#include <Histogram.h>

#include <pcl/common/transforms.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  UniquenessSelf<P, K, F>::UniquenessSelf()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void UniquenessSelf<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("uq_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void UniquenessSelf<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();
    bool write_histos = this->m_params->template get<bool>("uniqueness_self_write_histos", false);
    if(write_histos)
    {
      int uniqueness_k = this->m_params->template get<int>("uniqueness_k", 5);
      float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);

      for(int i = 0; i < uniqueness_k; ++i)
      {
        typename Histogram::Ptr hist(new Histogram(0.0, 6*model_resolution, 60, false));
        hist->setName("nn" + precision_cast(i, 1) + "-histogram");
        this->m_additional_logs.push_back(hist);
      }
    }
  }

  template<typename P, typename K, typename F>
  int UniquenessSelf<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif
    bool write_histos = this->m_params->template get<bool>("uniqueness_self_write_histos", false);
    if(write_histos)
    {
      int uniqueness_k = this->m_params->template get<int>("uniqueness_k", 5);
      assert(this->m_additional_logs.size() == uniqueness_k);

      typename ContextType::KeypointCloud::Ptr keypoints = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints");

      typename pcl::search::KdTree<K>::Ptr keypoint_search(new typename pcl::search::KdTree<K>);
      keypoint_search->setInputCloud(keypoints);

      size_t c = keypoints->size();
      for (size_t i = 0; i < c; ++i)
      {
        K k = keypoints->points[i];

        std::vector<int> idx_target;
        std::vector<float> dst;
        keypoint_search->nearestKSearch(k, uniqueness_k + 1, idx_target, dst);

        for(size_t j = 0; j < uniqueness_k; ++j)
        {
          typename Histogram::Ptr hist = boost::dynamic_pointer_cast<Histogram>
                                         (this->m_additional_logs[j]);
          hist->add(dst[j+1], run);
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
  std::string UniquenessSelf<P, K, F>::getName() const
  {
    return "UQS";
  }
}

PCLREF_INSTANTIATE_PRODUCT(UniquenessSelf,PCLREF_TYPES_PRODUCT)
