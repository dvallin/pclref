#include <KeypointStatsSelf.h>
#include <DataTable.h>
#include <Arrays.h>
#include <Histogram.h>

#include <pcl/common/transforms.h>

namespace pclref
{

  template<typename P, typename K, typename F>
  KeypointStatsSelf<P, K, F>::KeypointStatsSelf()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void KeypointStatsSelf<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ks_time");
    names.push_back("ks_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void KeypointStatsSelf<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();
    bool write_histos = this->m_params->template get<bool>("ks_write_histos", false);
    if(write_histos)
    {
      int keypoint_stats_k = this->m_params->template get<int>("keypoint_stats_k", 5);
      float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);
      for(int i = 0; i < keypoint_stats_k; ++i)
      {
        typename Histogram::Ptr hist(new Histogram(0.0, 6*model_resolution, 60, false));
        hist->setName("random-nn" + precision_cast(i, 1) + "-histogram");
        this->m_additional_logs.push_back(hist);
      }
    }
  }

  template<typename P, typename K, typename F>
  int KeypointStatsSelf<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    int keypoint_stats_k = this->m_params->template get<int>("keypoint_stats_k", 5);

    typename ContextType::KeypointCloud::Ptr keypoints = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints");
    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");

    bool write_histos = this->m_params->template get<bool>("ks_write_histos", false);
    if(write_histos)
    {
      size_t c = keypoints->size();
      typename ContextType::PointCloud::Ptr samples = typename ContextType::PointCloud::Ptr(new typename ContextType::PointCloud);
      {
        std::vector<size_t> indices;
        Arrays::pick(0, cloud->size(), keypoints->size(), indices);
        BOOST_FOREACH(size_t idx, indices)
        {
          samples->points.push_back(cloud->points[idx]);
        }
      }

      typename pcl::search::KdTree<P>::Ptr tree(new typename pcl::search::KdTree<P>);
      tree->setInputCloud(samples);

      for (size_t i = 0; i < c; ++i)
      {
        P p = samples->points[i];
        if(!isValid(p))
          continue;

        std::vector<int> idx_target;
        std::vector<float> dst;
        tree->nearestKSearch(p, keypoint_stats_k + 1, idx_target, dst);

        for(size_t j = 0; j < keypoint_stats_k; ++j)
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
    this->m_logger->set(run, 1, keypoints->points.size());

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string KeypointStatsSelf<P, K, F>::getName() const
  {
    return "KSS";
  }
}

PCLREF_INSTANTIATE_PRODUCT(KeypointStatsSelf,PCLREF_TYPES_PRODUCT)
