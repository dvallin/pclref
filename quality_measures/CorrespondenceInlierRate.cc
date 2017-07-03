#include <CorrespondenceInlierRate.h>
#include <DataTable.h>
#include <Arrays.h>

#include <pcl/registration/correspondence_estimation.h>

namespace pclref
{

  template<typename P, typename K, typename F>
  CorrespondenceInlierRate<P, K, F>::CorrespondenceInlierRate()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void CorrespondenceInlierRate<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ci_time");
    names.push_back("ci_cr");
    names.push_back("ci_ci");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int CorrespondenceInlierRate<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    int hits = 0;
    if(context->exists("correspondences_unfiltered") || context->exists("correspondences"))
    {
      typename ContextType::PointCloud::Ptr cloud_trans_source(new typename ContextType::PointCloud);
      typename ContextType::KeypointCloud::Ptr keypoints_trans_source(new typename ContextType::KeypointCloud);
      typename ContextType::KeypointCloud::Ptr keypoints_trans_target(new typename ContextType::KeypointCloud);
      // on overlapping keypoints or repeatable keypoints!
      typename ContextType::IndexCloud::Ptr rep_keypoints = context->template get<typename ContextType::IndexCloud::Ptr>("keypoints_unique_rep");
      typename ContextType::PointCloud::Ptr cloud_source = context->template get<typename ContextType::PointCloud::Ptr>("cloud_source");
      typename ContextType::KeypointCloud::Ptr keypoints_source = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_source");
      typename ContextType::KeypointCloud::Ptr keypoints_target = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_target");
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_source = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_target = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
      pcl::transformPointCloud(*keypoints_source, *keypoints_trans_source, *ground_truth_source);
      pcl::transformPointCloud(*keypoints_target, *keypoints_trans_target, *ground_truth_target);
      pcl::transformPointCloud(*cloud_source, *cloud_trans_source, *ground_truth_source);

      std::map<int, int> gt_correspondences;
      boost::shared_ptr<pcl::Correspondences> correspondences = context->template get<boost::shared_ptr<pcl::Correspondences> >("correspondences_unfiltered", "correspondences");

      float epsilon = this->m_params->template get<double>("gt_correspondences_epsilon", 0.15);

      typename pcl::search::KdTree<K>::Ptr keypoint_search_source(new typename pcl::search::KdTree<K>);
      typename pcl::search::KdTree<K>::Ptr keypoint_search_target(new typename pcl::search::KdTree<K>);
      keypoint_search_source->setInputCloud(keypoints_trans_source);
      keypoint_search_target->setInputCloud(keypoints_trans_target);

      for(size_t i = 0; i < rep_keypoints->points.size(); ++i)
      {
        P p = cloud_trans_source->points[rep_keypoints->points[i]];
        K kp;
        kp.x = p.x;
        kp.y = p.y;
        kp.z = p.z;

        std::vector<int> idx_source;
        std::vector<float> dst_source;
        std::vector<int> idx_target;
        std::vector<float> dst_target;

        keypoint_search_source->nearestKSearch(kp, 1, idx_source, dst_source);
        keypoint_search_target->nearestKSearch(kp, 1, idx_target, dst_target);

        if(dst_target[0] < epsilon*epsilon)
          gt_correspondences.insert(std::make_pair<int, int>(idx_source[0], idx_target[0]));
      }

      BOOST_FOREACH(pcl::Correspondence& corr, *correspondences)
      {
        if(gt_correspondences[corr.index_query] == corr.index_match)
          hits++;
      }

      this->m_logger->set(run, 1, rep_keypoints->size() > 0 ? ((float)hits)/rep_keypoints->size() : 0.0f);
      this->m_logger->set(run, 2, ((float)hits)/correspondences->size());
    }
    else
    {
      this->m_logger->set(run, 1, 0.0f);
      this->m_logger->set(run, 2, 0.0f);
    }
#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif
    this->m_logger->set(run, 0, ms);

    return hits;
  }

  template<typename P, typename K, typename F>
  std::string CorrespondenceInlierRate<P, K, F>::getName() const
  {
    return "CI";
  }
}

PCLREF_INSTANTIATE_PRODUCT(CorrespondenceInlierRate,PCLREF_TYPES_PRODUCT)
