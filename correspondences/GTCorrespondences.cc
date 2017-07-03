#include <GTCorrespondences.h>
#include <DataTable.h>
#include <Arrays.h>

#include <pcl/registration/correspondence_estimation.h>

namespace pclref
{

  template<typename P, typename K, typename F>
  GTCorrespondences<P, K, F>::GTCorrespondences()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void GTCorrespondences<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("gc_time");
    names.push_back("gc_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int GTCorrespondences<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::KeypointCloud::Ptr keypoints_trans_source(new typename ContextType::KeypointCloud);
    typename ContextType::KeypointCloud::Ptr keypoints_trans_target(new typename ContextType::KeypointCloud);
    typename ContextType::KeypointCloud::Ptr keypoints_source = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_source");
    typename ContextType::KeypointCloud::Ptr keypoints_target = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_target");
    boost::shared_ptr<typename ContextType::Transformation> ground_truth_source = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
    boost::shared_ptr<typename ContextType::Transformation> ground_truth_target = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
    pcl::transformPointCloud(*keypoints_target, *keypoints_trans_target, *ground_truth_target);
    pcl::transformPointCloud(*keypoints_source, *keypoints_trans_source, *ground_truth_source);

    boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);

    bool random_sample = this->m_params->template get<bool>("gt_correspondences_random_sample", false);
    float epsilon = this->m_params->template get<double>("gt_correspondences_epsilon", 0.15);

    if(!random_sample)
    {
      typename pcl::search::KdTree<K>::Ptr keypoint_search(new typename pcl::search::KdTree<K>);
      keypoint_search->setInputCloud(keypoints_trans_target);

      for(size_t i = 0; i < keypoints_trans_source->points.size(); ++i)
      {
        K k = keypoints_trans_source->points[i];
        std::vector<int> idx_target;
        std::vector<float> dst;
        keypoint_search->nearestKSearch(k, 5, idx_target, dst);
        for(size_t j = 0; j < dst.size(); ++j)
          if(dst[j] < epsilon*epsilon)
          {
            correspondences->push_back(pcl::Correspondence(i, idx_target[j], dst[j]));
          }
      }
    }
    else
    {
      for(size_t i = 0; i < keypoints_trans_source->points.size(); ++i)
      {
        K k0 = keypoints_trans_source->points[i];
        for(size_t j = 0; j < keypoints_trans_target->points.size(); ++j)
        {
          K k1 = keypoints_trans_target->points[j];
          double dist = distance(k0, k1, pcl::L2);
          if(dist < epsilon)
          {
            correspondences->push_back(pcl::Correspondence(i, j, dist));
          }
        }
      }
      while(correspondences->size() > 2*std::min(keypoints_trans_source->size(),
            keypoints_trans_target->size()))
      {
        size_t r = MersenneTwister::nextUint(0, correspondences->size());
        correspondences->operator [](r) = correspondences->operator [](correspondences->size() - 1);
        correspondences->pop_back();
      }

    }
    context->set("correspondences", correspondences);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, correspondences->size());

    return correspondences->size();
  }

  template<typename P, typename K, typename F>
  std::string GTCorrespondences<P, K, F>::getName() const
  {
    return "GC";
  }
}

PCLREF_INSTANTIATE_PRODUCT(GTCorrespondences,PCLREF_TYPES_PRODUCT)
