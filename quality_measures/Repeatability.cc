#include <Repeatability.h>
#include <DataTable.h>

#include <pcl/common/transforms.h>

namespace pclref
{

  template<typename P, typename K, typename F>
  Repeatability<P, K, F>::Repeatability()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void Repeatability<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("re_time");
    names.push_back("re_size");
    names.push_back("re_rep");
    names.push_back("re_s_rep");
    names.push_back("re_u_rep");
    names.push_back("re_us_rep");
    names.push_back("re_r_rep");
    names.push_back("re_rs_rep");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int  Repeatability<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif
    typename ContextType::IndexCloud::Ptr keypoints_overlap = context->template get<typename ContextType::IndexCloud::Ptr>("keypoints_overlap");
    typename ContextType::IndexCloud::Ptr keypoint_indices_source = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices_source");

    typename ContextType::KeypointCloud::Ptr keypoints_target = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_target");
    typename ContextType::KeypointCloud::Ptr keypoints_source = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_source");
    double scale_rep = 0;
    double unique_scale_rep = 0;
    double reciprocal_scale_rep = 0;
    size_t runs = 0;
    typename ContextType::IndexCloud::Ptr keypoints_rep(new typename ContextType::IndexCloud);
    typename ContextType::IndexCloud::Ptr keypoints_unique_rep(new typename ContextType::IndexCloud);
    typename ContextType::IndexCloud::Ptr keypoints_reciprocal_rep(new typename ContextType::IndexCloud);

    if(keypoints_overlap->size() > 0
        && keypoints_target->size() > 0
        && keypoints_source->size() > 0)
    {
      typename ContextType::PointCloud::Ptr cloud_source = context->template get<typename ContextType::PointCloud::Ptr>("cloud_source");
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_source = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_target = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
      typename ContextType::PointCloud::Ptr cloud_trans_source(new typename ContextType::PointCloud);
      typename ContextType::KeypointCloud::Ptr keypoints_trans_target(new typename ContextType::KeypointCloud);
      typename ContextType::KeypointCloud::Ptr keypoints_trans_source(new typename ContextType::KeypointCloud);
      pcl::transformPointCloud(*cloud_source, *cloud_trans_source, *ground_truth_source);
      pcl::transformPointCloud(*keypoints_source, *keypoints_trans_source, *ground_truth_source);
      pcl::transformPointCloud(*keypoints_target, *keypoints_trans_target, *ground_truth_target);

      typename pcl::search::KdTree<K>::Ptr keypoint_search(new typename pcl::search::KdTree<K>);
      typename pcl::search::KdTree<K>::Ptr keypoint_inv_search(new typename pcl::search::KdTree<K>);
      float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);
      keypoint_search->setInputCloud(keypoints_trans_target);
      keypoint_inv_search->setInputCloud(keypoints_trans_source);

      // repeatability averaged over epsilons in interval(epsilon_min, epsilon_max, epsilon_step)
      double epsilon_min, epsilon_max, epsilon_step;
      bool repeatability_range = this->m_params->template get<bool>("repeatability_range", false);
      if(repeatability_range)
      {
        epsilon_min = this->m_params->template get<double>("repeatability_epsilon_min", 0.5*model_resolution);
        epsilon_max = this->m_params->template get<double>("repeatability_epsilon_max", 3*model_resolution);
        epsilon_step = this->m_params->template get<double>("repeatability_epsilon_step", 0.5*model_resolution);
      }
      else
      {
        double epsilon = this->m_params->template get<double>("repeatability_epsilon", 1.5*model_resolution);
        epsilon_min = epsilon;
        epsilon_max = epsilon;
        epsilon_step = 1;
      }
      epsilon_max += std::numeric_limits<double>::epsilon();

      double theta_scale = this->m_params->template get<double>("repeatability_theta_scale", 1.0);

      for(double epsilon = epsilon_min; epsilon <= epsilon_max; epsilon += epsilon_step, ++runs)
      {
        double sr, usr, rsr;
        double theta = epsilon*theta_scale;
        calculateRepeatability(keypoints_overlap, keypoint_indices_source,
                               keypoint_search, keypoint_inv_search,
                               cloud_trans_source, keypoints_trans_target,
                               epsilon, theta,
                               keypoints_rep, sr,
                               keypoints_unique_rep, usr,
                               keypoints_reciprocal_rep, rsr);
        scale_rep += sr;
        unique_scale_rep += usr;
        reciprocal_scale_rep += rsr;
      }
    }
    context->set("keypoints_rep", keypoints_rep);
    context->set("keypoints_unique_rep", keypoints_unique_rep);
    context->set("keypoints_reciprocal_rep", keypoints_reciprocal_rep);

    // average over all runs averaged over size of overlap
    size_t c = runs*keypoints_overlap->size();
    c = c > 0u ? c : 1;
    this->m_logger->set(run, 1, keypoints_rep->size());
    this->m_logger->set(run, 2, (float)keypoints_rep->size() / c);
    this->m_logger->set(run, 3, scale_rep / c);
    this->m_logger->set(run, 4, (float)keypoints_unique_rep->size() / c);
    this->m_logger->set(run, 5, unique_scale_rep / c);
    this->m_logger->set(run, 6, (float)keypoints_reciprocal_rep->size() / c);
    this->m_logger->set(run, 7, reciprocal_scale_rep / c);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return keypoints_rep->size();
  }

  template<typename P, typename K, typename F>
  std::string Repeatability<P, K, F>::getName() const
  {
    return "RE";
  }

  template<typename P, typename K, typename F>
  void Repeatability<P, K, F>::calculateRepeatability(
    typename ContextType::IndexCloud::Ptr keypoints_overlap,
    typename ContextType::IndexCloud::Ptr keypoint_indices_source,
    typename pcl::search::KdTree<K>::Ptr keypoint_search,
    typename pcl::search::KdTree<K>::Ptr keypoint_inv_search,
    typename ContextType::PointCloud::Ptr cloud_trans_source,
    typename ContextType::KeypointCloud::Ptr keypoints_trans_target,
    double epsilon, double theta,
    typename ContextType::IndexCloud::Ptr keypoints_rep,
    double& scale_rep,
    typename ContextType::IndexCloud::Ptr keypoints_unique_rep,
    double& unique_scale_rep,
    typename ContextType::IndexCloud::Ptr keypoints_reciprocal_rep,
    double& reciprocal_scale_rep)
  {
    scale_rep = 0.0;
    unique_scale_rep = 0.0;
    size_t c = keypoints_overlap->size();

    for (size_t i = 0; i < c; ++i)
    {
      int idx = keypoints_overlap->points[i];
      P p = cloud_trans_source->points[idx];
      if(!isValid(p))
        continue;

      K k;
      k.x = p.x;
      k.y = p.y;
      k.z = p.z;

      std::vector<int> idx_target;
      std::vector<float> dst;
      keypoint_search->nearestKSearch(k, 2, idx_target, dst);
      if(dst[0] < epsilon*epsilon)
      {
        // calculate normalized intersection volume of spheres (|intersection|/|union|)
        // A, the volume of one sphere, is 4/3 PI r^3
        // B, the volume of sphere cap, is (PI h^2)/3 (3r - h) (look that shit up on wikipedia)
        // with r = epsilon/2 and h = sqrt(r^2 - (d/2)^2)
        // so intersection is 2B and union is 2A-2B
        // rep = intersection/union = 2B / (2A-2B) = B / (A-B)
        // note for A and B PI/3 has been eliminated.
        double r = 0.5*epsilon;
        double h = sqrt(r*r - 0.25*dst[0]);
        double A = 4*r*r*r;
        double B = h*h * (3*r - h);
        double rep = B / (A - B);

        keypoints_rep->points.push_back(idx);
        scale_rep += rep;
        if(dst[1] >= theta*theta)
        {
          keypoints_unique_rep->points.push_back(idx);
          unique_scale_rep += rep;
        }

        std::vector<int> idx_inv;
        std::vector<float> dst_inv;
        K k_inv = keypoints_trans_target->points[idx_target[0]];
        keypoint_inv_search->nearestKSearch(k_inv, 2, idx_inv, dst_inv);
        int src_idx = keypoint_indices_source->points[idx_inv[0]];
        if(src_idx == idx)
        {
          keypoints_reciprocal_rep->points.push_back(idx);
          reciprocal_scale_rep += rep;
        }
      }
    }
  }

}

PCLREF_INSTANTIATE_PRODUCT(Repeatability,PCLREF_TYPES_PRODUCT)
