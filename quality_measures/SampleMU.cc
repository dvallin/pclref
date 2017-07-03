#include <SampleMU.h>
#include <DataTable.h>
#include <ProcessingStep.h>
#include <Histogram.h>
#include <MersenneTwister.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  SampleMU<P, K, F>::SampleMU()
  {
#ifdef PCLREF_PARALLEL
    omp_init_lock(&m_writelock);
#endif
    init();
  }

  template<typename P, typename K, typename F>
  SampleMU<P, K, F>::~SampleMU()
  {
#ifdef PCLREF_PARALLEL
    omp_destroy_lock(&m_writelock);
#endif
  }

  template<typename P, typename K, typename F>
  void SampleMU<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("mu_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void SampleMU<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();

    bool write_sets = this->m_params->template get<bool>("mu_write_sets", false);
    bool write_features = this->m_params->template get<bool>("mu_write_features", false);

    if(write_sets)
    {
      std::vector<DataTable::columnType> types;
      types.push_back(DataTable::ENUM);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      std::vector<std::string> names;
      names.push_back("set");
      names.push_back("L2_keypoints");
      names.push_back("L1_features");
      names.push_back("L2_features");
      names.push_back("L2SQR_features");
      names.push_back("LINF_features");
      //names.push_back("JM_features"); coordinates might be negative -> NAN
      names.push_back("B_features");
      names.push_back("SUBLINEAR_features");
      names.push_back("CS_features");
      names.push_back("DIV_features");
      //names.push_back("PF_features"); additional parameters needed
      //names.push_back("K_features"); additional parameters needed
      names.push_back("KL_features");
      names.push_back("HIK_features");

      if(write_features)
      {
        for(int i = 0; i < pclDataLength<typename F::Type>(); ++i)
        {
          types.push_back(DataTable::FLOAT);
          names.push_back("l" + precision_cast(i, 1));
        }
        for(int i = 0; i < pclDataLength<typename F::Type>(); ++i)
        {
          types.push_back(DataTable::FLOAT);
          names.push_back("r" + precision_cast(i, 1));
        }
      }

      typename DataTable::Ptr log = typename DataTable::Ptr(new DataTable(types, names));
      log->setName("um_log");
      this->m_additional_logs.push_back(log);
    }
  }

  template<typename P, typename K, typename F>
  int SampleMU<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::KeypointCloud::Ptr keypoints_target = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_target");
    boost::shared_ptr<pcl::Correspondences> m_set(new pcl::Correspondences);
    boost::shared_ptr<pcl::Correspondences> u_set(new pcl::Correspondences);

    typename ContextType::FeatureCloud::Ptr features_source = context->template get<typename ContextType::FeatureCloud::Ptr>("features_source");
    typename ContextType::FeatureCloud::Ptr features_target = context->template get<typename ContextType::FeatureCloud::Ptr>("features_target");

    // if there is no overlap or no target keypoints mset cannot be created
    // Note there may be overlap and no target keypoints, as overlaping keypoints is not defined in respect to the target keypoints,
    // repeatability does this!
    typename ContextType::IndexCloud::Ptr keypoints_overlap = context->template get<typename ContextType::IndexCloud::Ptr>("keypoints_overlap");
    if(keypoints_overlap->size() * keypoints_target->size() > 0)
    {
      typename ContextType::PointCloud::Ptr cloud_source = context->template get<typename ContextType::PointCloud::Ptr>("cloud_source");
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_source = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_target = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
      typename ContextType::PointCloud::Ptr cloud_trans_source(new typename ContextType::PointCloud);
      typename ContextType::KeypointCloud::Ptr keypoints_trans_target(new typename ContextType::KeypointCloud);
      pcl::transformPointCloud(*cloud_source, *cloud_trans_source, *ground_truth_source);
      pcl::transformPointCloud(*keypoints_target, *keypoints_trans_target, *ground_truth_target);

      typename pcl::search::KdTree<K>::Ptr keypoint_search(new typename pcl::search::KdTree<K>);
      keypoint_search->setInputCloud(keypoints_trans_target);

      typename ContextType::IndexCloud::Ptr keypoint_indices_source = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices_source");

      std::map<int, int> keypoint_source_mapping;
      for(int i = 0; i < keypoint_indices_source->size(); ++i)
        keypoint_source_mapping.insert(std::pair<int,int>(keypoint_indices_source->points[i], i));

      float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);
      double epsilon = this->m_params->template get<double>("m_u_epsilon", 2*model_resolution);
      double epsilon_sqr = epsilon*epsilon;

      size_t c = keypoints_overlap->size();

      for (size_t i = 0; i < c; ++i)
      {
        int idx = keypoints_overlap->points[i];
        P p = cloud_trans_source->points[idx];
        assert(isValid(p));

        K k;
        k.x = p.x;
        k.y = p.y;
        k.z = p.z;

        std::vector<int> idx_target;
        std::vector<float> dst;
        keypoint_search->nearestKSearch(k, 5, idx_target, dst);

        for(int i = 0; i < idx_target.size(); ++i)
        {
          K k2_target = keypoints_trans_target->points[idx_target[i]];
          assert(isValid(k2_target));

          if(dst[i] < epsilon_sqr)
          {
            // found a neat sample for the M set
            m_set->push_back(pcl::Correspondence(keypoint_source_mapping[idx], idx_target[i], dst[i]));
          }
          else if(u_set->size() < m_set->size())
          {
            std::vector<int> idx_target2;
            float d2 = -1;

            int tries = 5;
            while(--tries >= 0)
            {
              // randomly pick another keypoint to match this keypoint to
              int kidx = MersenneTwister::nextUint(0, c);
              int idx2 = keypoints_overlap->points[kidx];
              P p2 = cloud_trans_source->points[idx2];
              assert(isValid(p2));

              // find a partner of this keypoint in target cloud
              K k2;
              k2.x = p2.x;
              k2.y = p2.y;
              k2.z = p2.z;
              std::vector<float> dst2;
              idx_target2.clear();
              keypoint_search->nearestKSearch(k2, 1, idx_target2, dst2);

              K k2_other = keypoints_trans_target->points[idx_target2[0]];
              assert(isValid(k2_other));

              // only match it if it is not in nearest neighbours of k
              bool match = idx_target2[0] != idx;
              for(int j = 0; j < 5; ++j)
                match &= idx_target[i] != idx_target2[0];
              d2 = pcl::selectNorm(k2_other.data, p.data, 3, pcl::L2_SQR);
              if(match && d2 > epsilon_sqr)
                break;
            }
            if(tries >= 0) u_set->push_back(pcl::Correspondence(keypoint_source_mapping[idx], idx_target2[0], d2));
          }
        }
      }

      analyze(features_source, features_target, m_set, u_set);
    }

    typename ContextType::FeatureCloud::Ptr mCloud(new typename ContextType::FeatureCloud);
    typename ContextType::FeatureCloud::Ptr uCloud(new typename ContextType::FeatureCloud);

    BOOST_FOREACH(pcl::Correspondence corr, *m_set)
    {
      Eigen::VectorXd l = pclref::toEigenVector(features_source->points[corr.index_query]);
      Eigen::VectorXd r = pclref::toEigenVector(features_target->points[corr.index_match]);
      mCloud->push_back(pclref::fromEigenVector<typename F::Type, double>(r - l));
      mCloud->push_back(pclref::fromEigenVector<typename F::Type, double>(l - r));
    }
    BOOST_FOREACH(pcl::Correspondence corr, *u_set)
    {
      Eigen::VectorXd l = pclref::toEigenVector(features_source->points[corr.index_query]);
      Eigen::VectorXd r = pclref::toEigenVector(features_target->points[corr.index_match]);
      uCloud->push_back(pclref::fromEigenVector<typename F::Type, double>(r - l));
      uCloud->push_back(pclref::fromEigenVector<typename F::Type, double>(l - r));
    }
    context->add("match_feature_diffs", mCloud);
    context->add("unmatch_feature_diffs", uCloud);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string SampleMU<P, K, F>::getName() const
  {
    return "MU";
  }

  template<typename P, typename K, typename F>
  template<typename T>
  void SampleMU<P, K, F>::analyze(
    boost::shared_ptr<pcl::PointCloud<T> > features_source,
    boost::shared_ptr<pcl::PointCloud<T> > features_target,
    boost::shared_ptr<pcl::Correspondences> m_set,
    boost::shared_ptr<pcl::Correspondences> u_set)
  {
    if(m_set->size()*u_set->size() == 0)
      return;

    bool write_sets = this->m_params->template get<bool>("mu_write_sets", false);
    bool write_features = this->m_params->template get<bool>("mu_write_features", false);
    int max_rows = this->m_params->template get<int>("mu_max_rows", 9999);

    if(write_sets)
    {
#ifdef PCLREF_PARALLEL
      omp_set_lock(&m_writelock);
#endif
      size_t row = this->m_additional_logs[0]->getRowCount();
      if(row < max_rows)
      {
        ((DataTable*)this->m_additional_logs[0].get())->_expandBy(2*m_set->size() + 2*u_set->size());
        {
          BOOST_FOREACH(pcl::Correspondence corr, *m_set)
          {
            T f_source = features_source->points[corr.index_query];
            T f_target = features_target->points[corr.index_match];

            this->m_additional_logs[0]->set(row, 0, "m");
            this->m_additional_logs[0]->set(row, 1, corr.distance);
            writeFeaturePair(row, f_source, f_target, write_features);
            ++row;

            this->m_additional_logs[0]->set(row, 0, "m");
            this->m_additional_logs[0]->set(row, 1, corr.distance);
            writeFeaturePair(row, f_target, f_source, write_features);
            ++row;
          }
        }
        {
          BOOST_FOREACH(pcl::Correspondence corr, *u_set)
          {
            T f_source = features_source->points[corr.index_query];
            T f_target = features_target->points[corr.index_match];

            this->m_additional_logs[0]->set(row, 0, "u");
            this->m_additional_logs[0]->set(row, 1, corr.distance);
            writeFeaturePair(row, f_source, f_target, write_features);
            ++row;

            this->m_additional_logs[0]->set(row, 0, "u");
            this->m_additional_logs[0]->set(row, 1, corr.distance);
            writeFeaturePair(row, f_target, f_source, write_features);
            ++row;
          }
        }
      }
#ifdef PCLREF_PARALLEL
      omp_unset_lock(&m_writelock);
#endif
    }
  }

  template<typename P, typename K, typename F>
  template<typename T>
  void SampleMU<P, K, F>::writeFeaturePair(size_t row, const T& f_source, const T& f_target, bool write_features)
  {
    this->m_additional_logs[0]->set(row, 2, distance(f_source, f_target, pcl::L1));
    this->m_additional_logs[0]->set(row, 3, distance(f_source, f_target, pcl::L2));
    this->m_additional_logs[0]->set(row, 4, distance(f_source, f_target, pcl::L2_SQR));
    this->m_additional_logs[0]->set(row, 5, distance(f_source, f_target, pcl::LINF));
    this->m_additional_logs[0]->set(row, 6, distance(f_source, f_target, pcl::B));
    this->m_additional_logs[0]->set(row, 7, distance(f_source, f_target, pcl::SUBLINEAR));
    this->m_additional_logs[0]->set(row, 8, distance(f_source, f_target, pcl::CS));
    this->m_additional_logs[0]->set(row, 9, distance(f_source, f_target, pcl::DIV));
    this->m_additional_logs[0]->set(row, 10, distance(f_source, f_target, pcl::KL));
    this->m_additional_logs[0]->set(row, 11, distance(f_source, f_target, pcl::HIK));

    if(write_features)
    {
      const size_t f_length = pclDataLength<T>();
      const float* f_data = pclData(f_source);
      for(int i = 0; i < f_length; ++i)
      {
        this->m_additional_logs[0]->set(row, 12 + i, f_data[i]);
      }
    }
    if(write_features)
    {
      const size_t f_length = pclDataLength<T>();
      const float* f_data = pclData(f_target);
      for(int i = 0; i < f_length; ++i)
      {
        this->m_additional_logs[0]->set(row, 12 + f_length + i, f_data[i]);
      }
    }
  }
}

PCLREF_INSTANTIATE_PRODUCT(SampleMU,PCLREF_TYPES_PRODUCT)
