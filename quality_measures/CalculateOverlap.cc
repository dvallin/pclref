#include <CalculateOverlap.h>
#include <DataTable.h>

#include <pcl/common/transforms.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  CalculateOverlap<P, K, F>::CalculateOverlap()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void CalculateOverlap<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("co_time");
    names.push_back("co_cloud_size");
    names.push_back("co_keypoint_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int CalculateOverlap<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    bool overlap_on_keypoints = this->m_params->template get<bool>("overlap_on_keypoints", true);
    float overlap_threshold = this->m_params->template get<double>("overlap_threshold", 0.1);

    float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);

    typename ContextType::PointCloud::Ptr cloud_source = context->template get<typename ContextType::PointCloud::Ptr>("cloud_source");
    typename ContextType::PointCloud::Ptr cloud_target = context->template get<typename ContextType::PointCloud::Ptr>("cloud_target");
    boost::shared_ptr<typename ContextType::Transformation> ground_truth_source = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
    boost::shared_ptr<typename ContextType::Transformation> ground_truth_target = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
    typename ContextType::PointCloud::Ptr cloud_trans_source(new typename ContextType::PointCloud);
    typename ContextType::PointCloud::Ptr cloud_trans_target(new typename ContextType::PointCloud);
    pcl::transformPointCloud(*cloud_source, *cloud_trans_source, *ground_truth_source);
    pcl::transformPointCloud(*cloud_target, *cloud_trans_target, *ground_truth_target);

    typename pcl::search::KdTree<P>::Ptr tree(new typename pcl::search::KdTree<P>);
    tree->setInputCloud(cloud_trans_target);

    typename ContextType::IndexCloud::Ptr cloud_overlap(new typename ContextType::IndexCloud);

    double epsilon = this->m_params->template get<double>("overlap_point_epsilon", model_resolution);

    for (size_t i = 0; i < cloud_source->size(); ++i)
    {
      std::vector<int> idx(1);
      std::vector<float> dst(1);
      P p = cloud_trans_source->points[i];
      if(!isValid(p))
        continue;

      tree->nearestKSearch(p, 1, idx, dst);
      if (dst[0] < epsilon*epsilon)
      {
        cloud_overlap->push_back(i);
      }
    }
    context->set("cloud_overlap", cloud_overlap);
    double overlap_ratio = (float)cloud_overlap->size() / cloud_source->size();
    this->m_logger->set(run, 1, overlap_ratio);

    if(overlap_on_keypoints)
    {
      typename ContextType::IndexCloud::Ptr cloud_overlap =  context->template get<typename ContextType::IndexCloud::Ptr>("cloud_overlap");
      typename ContextType::IndexCloud::Ptr keypoints_source = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices_source");

      typename ContextType::IndexCloud::Ptr keypoints_overlap(new typename ContextType::IndexCloud);

      std::set<int> kps;
      kps.insert(keypoints_source->points.begin(), keypoints_source->points.end());

      for (size_t i = 0; i < cloud_overlap->size(); ++i)
      {
        int idx = cloud_overlap->points[i];
        if(kps.find(idx) != kps.end())
        {
          keypoints_overlap->points.push_back(idx);
        }
      }
      context->set("keypoints_overlap", keypoints_overlap);
      this->m_logger->set(run, 2, (float)keypoints_overlap->size() / keypoints_source->size());
    }
    else
    {
      this->m_logger->set(run, 2, -1);
    }

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return overlap_ratio > overlap_threshold ? 1 : 0;
  }

  template<typename P, typename K, typename F>
  std::string CalculateOverlap<P, K, F>::getName() const
  {
    return "CO";
  }
}

PCLREF_INSTANTIATE_PRODUCT(CalculateOverlap,PCLREF_TYPES_PRODUCT)
