#include <EstimateCorrespondences.h>
#include <DataTable.h>

#include <pcl/registration/correspondence_estimation.h>
#include <correspondence_estimation_improved.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  EstimateCorrespondences<P, K, F>::EstimateCorrespondences()
  {
#ifdef PCLREF_PARALLEL
    omp_init_lock(&m_writelock);
#endif
    init();
  }

  template<typename P, typename K, typename F>
  EstimateCorrespondences<P, K, F>::~EstimateCorrespondences()
  {
#ifdef PCLREF_PARALLEL
    omp_destroy_lock(&m_writelock);
#endif
  }

  template<typename P, typename K, typename F>
  void EstimateCorrespondences<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ec_time");
    names.push_back("ec_size");
    names.push_back("ec_ssd");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void EstimateCorrespondences<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();

    bool write_sets = this->m_params->template get<bool>("estimate_correspondences_write_sets", false);
    bool write_features = this->m_params->template get<bool>("estimate_correspondences_write_features", false);

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
  int EstimateCorrespondences<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
    double ssd;
    typename ContextType::FeatureCloud::Ptr features_source = context->template get<typename ContextType::FeatureCloud::Ptr>("features_source");
    typename ContextType::FeatureCloud::Ptr features_target = context->template get<typename ContextType::FeatureCloud::Ptr>("features_target");
    if(features_source->size() * features_target->size() > 0)
    {
      estimate(context, features_source, features_target, correspondences);
      ssd = analyze(context, features_source, features_target, correspondences);
    }
    context->set("correspondences", correspondences);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, correspondences->size());
    this->m_logger->set(run, 2, ssd);

    return correspondences->size();
  }

  template<typename P, typename K, typename F>
  std::string EstimateCorrespondences<P, K, F>::getName() const
  {
    return "EC";
  }


  template<typename P, typename K, typename F>
  template<typename T>
  double EstimateCorrespondences<P, K, F>::analyze(typename ContextType::Ptr context,
      boost::shared_ptr<pcl::PointCloud<T> > features_source,
      boost::shared_ptr<pcl::PointCloud<T> > features_target,
      boost::shared_ptr<pcl::Correspondences> correspondences)
  {
    bool write_sets = this->m_params->template get<bool>("estimate_correspondences_write_sets", false);
    bool write_features = this->m_params->template get<bool>("estimate_correspondences_write_features", false);

    typename ContextType::KeypointCloud::Ptr keypoints_source = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_source");
    typename ContextType::KeypointCloud::Ptr keypoints_target = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_target");
    boost::shared_ptr<typename ContextType::Transformation> ground_truth_source = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
    boost::shared_ptr<typename ContextType::Transformation> ground_truth_target = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
    typename ContextType::KeypointCloud::Ptr keypoints_trans_source(new typename ContextType::KeypointCloud);
    typename ContextType::KeypointCloud::Ptr keypoints_trans_target(new typename ContextType::KeypointCloud);
    pcl::transformPointCloud(*keypoints_source, *keypoints_trans_source, *ground_truth_source);
    pcl::transformPointCloud(*keypoints_target, *keypoints_trans_target, *ground_truth_target);
    assert(keypoints_source->size() == keypoints_trans_source->size());
    assert(keypoints_target->size() == keypoints_trans_target->size());
    double ssd = 0;

    size_t row;
    int max_rows = this->m_params->template get<int>("ec_max_rows", 9999);
    if(write_sets)
    {
#ifdef PCLREF_PARALLEL
      omp_set_lock(&m_writelock);
#endif
      row = this->m_additional_logs[0]->getRowCount();
      if(row < max_rows)
      {
        ((DataTable*)this->m_additional_logs[0].get())->_expandBy(correspondences->size());
      }
      else
      {
#ifdef PCLREF_PARALLEL
        omp_unset_lock(&m_writelock);
#endif
        write_sets = false;
      }
    }


    BOOST_FOREACH(pcl::Correspondence corr, *correspondences)
    {
      T f_source = features_source->points[corr.index_query];
      T f_target = features_target->points[corr.index_match];
      K k_source = keypoints_trans_source->points[corr.index_query];
      K k_target = keypoints_trans_target->points[corr.index_match];

      assert(isValid(k_source) && isValid(k_target));

      ssd += distance(k_source, k_target, pcl::L2_SQR);

      if(write_sets)
      {
        this->m_additional_logs[0]->set(row, 0, "c");
        this->m_additional_logs[0]->set(row, 1, distance(k_source, k_target, pcl::L2));
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
            this->m_additional_logs[0]->set(row, 11 + i, f_data[i]);
          }
        }
        if(write_features)
        {
          const size_t f_length = pclDataLength<T>();
          const float* f_data = pclData(f_target);
          for(int i = 0; i < f_length; ++i)
          {
            this->m_additional_logs[0]->set(row, 11 + f_length + i, f_data[i]);
          }
        }
        ++row;
      }
    }
#ifdef PCLREF_PARALLEL
    if(write_sets)
    {
      omp_unset_lock(&m_writelock);
    }
#endif
    return ssd;
  }

  template<typename P, typename K, typename F>
  template<typename T>
  void EstimateCorrespondences<P, K, F>::estimate(typename ContextType::Ptr context,
      boost::shared_ptr<pcl::PointCloud<T> > features_source,
      boost::shared_ptr<pcl::PointCloud<T> > features_target,
      boost::shared_ptr<pcl::Correspondences> correspondences)
  {
    bool reciprocal = this->m_params->template get<bool>("ec_reciprocal", true);
    int neighbours = this->m_params->template get<int>("ec_neighbours", 1);
    double keypoint_distance_threshold = this->m_params->template get<double>("ec_keypoint_distance_threshold",
        std::numeric_limits<double>::max());

    int feature_distance_metric = this->m_params->template get<int>("ec_feature_distance_metric", 4);
    switch(feature_distance_metric)
    {
    case 0:
      estimate_with_metric<T, flann::L2<float> >(features_source, features_target,
          *correspondences, keypoint_distance_threshold, reciprocal, neighbours);
      break;
    case 1:
      estimate_with_metric<T, flann::ChiSquareDistance<float> >(features_source, features_target,
          *correspondences, keypoint_distance_threshold, reciprocal, neighbours);
      break;
    case 2:
      estimate_with_metric<T, flann::L1<float> >(features_source, features_target,
          *correspondences, keypoint_distance_threshold, reciprocal, neighbours);
      break;
    case 3:
      estimate_with_metric<T, flann::HistIntersectionDistance<float> >(features_source, features_target,
          *correspondences, keypoint_distance_threshold, reciprocal, neighbours);
      break;
    case 4:
      estimate_with_metric<T, flann::HellingerDistance<float> >(features_source, features_target,
          *correspondences, keypoint_distance_threshold, reciprocal, neighbours);
      break;
    }
  }

  template<typename P, typename K, typename F>
  template<typename T, typename Metric>
  void EstimateCorrespondences<P, K, F>::estimate_with_metric(boost::shared_ptr<pcl::PointCloud<T> > features_source,
      boost::shared_ptr<pcl::PointCloud<T> > features_target,
      pcl::Correspondences& correspondences, double keypoint_distance_threshold, bool reciprocal, int neighbours)
  {
    typedef pcl::registration::CorrespondenceEstimationImproved<T, T, float, Metric> EstimatorT;
    EstimatorT estimator;
    estimator.setInputSource(features_source);
    estimator.setInputTarget(features_target);

    typename EstimatorT::IndexCreator flann_index_creator = (typename EstimatorT::IndexCreator)this->m_params->template get<int>("ec_flann_index_creator", 1);
    int flann_checks = this->m_params->template get<int>("ec_flann_checks", 256);
    double flann_epsilon = this->m_params->template get<double>("ec_flann_epsilon", 0.0f);
    estimator.setFlannParameters(flann_index_creator, flann_checks, flann_epsilon);

    switch(flann_index_creator)
    {
    case EstimatorT::MultiKDTree:
    {
      int trees = this->m_params->template get<int>("ec_flann_trees", 4);
      estimator.setKDForestParameters(trees);
    }
    break;
    case EstimatorT::Composite:
    {
      int trees = this->m_params->template get<int>("ec_flann_trees", 4);
      estimator.setKDForestParameters(trees);
    }
    case EstimatorT::KMeans:
    {
      int branching = this->m_params->template get<int>("ec_flann_branching", 32);
      int iterations = this->m_params->template get<int>("ec_flann_iterations", 11);
      flann::flann_centers_init_t centers = (flann::flann_centers_init_t)this->m_params->template get<int>("ec_flann_centers", 0);
      double cb_index = this->m_params->template get<double>("ec_flann_cb_index", 0.1);
      estimator.setKMeansParameters(branching, iterations, centers, cb_index);
    }
    break;
    case EstimatorT::Lsh:
    {
      int table_number = this->m_params->template get<int>("ec_flann_table_number", 20);
      int key_size = this->m_params->template get<int>("ec_flann_key_size", 16);
      int multi_probe_level = this->m_params->template get<int>("ec_flann_multi_probe_level", 2);
      estimator.setLshParameters(table_number, key_size, multi_probe_level);
    }
    break;
    case EstimatorT::Auto:
    {
      double target_precision = this->m_params->template get<double>("ec_flann_target_precision", 0.9);
      double build_weight = this->m_params->template get<double>("ec_flann_build_weight", 0.01);
      double memory_weight = this->m_params->template get<double>("ec_flann_memory_weight", 0);
      double sample_fraction = this->m_params->template get<double>("ec_flann_sample_fraction", 0.1);
      estimator.setAutoTuneParameters(target_precision, build_weight, memory_weight, sample_fraction);
    }
    break;
    }

    if (reciprocal)
    {
      estimator.determineReciprocalCorrespondences(correspondences, keypoint_distance_threshold, neighbours);
    }
    else
    {
      estimator.determineCorrespondences(correspondences, keypoint_distance_threshold, neighbours);
    }
  }
}

PCLREF_INSTANTIATE_PRODUCT(EstimateCorrespondences,PCLREF_TYPES_PRODUCT)
