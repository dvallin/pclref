#include <PredictCorrespondences.h>
#include <DataTable.h>

#ifdef PCLREF_OPENCV
namespace pclref
{
  template<typename P, typename K, typename F>
  PredictCorrespondences<P, K, F>::PredictCorrespondences()
  {
#ifdef PCLREF_PARALLEL
    omp_init_lock(&m_writelock);
#endif
    init();
  }

  template<typename P, typename K, typename F>
  PredictCorrespondences<P, K, F>::~PredictCorrespondences()
  {
#ifdef PCLREF_PARALLEL
    omp_destroy_lock(&m_writelock);
#endif
  }

  template<typename P, typename K, typename F>
  void PredictCorrespondences<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("pc_time");
    names.push_back("pc_size");
    names.push_back("pc_ssd");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void PredictCorrespondences<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();

    bool write_sets = this->m_params->template get<bool>("predict_correspondences_write_sets", true);

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

      typename DataTable::Ptr log = typename DataTable::Ptr(new DataTable(types, names));
      log->setName("um_log");
      this->m_additional_logs.push_back(log);
    }
  }

  template<typename P, typename K, typename F>
  int PredictCorrespondences<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
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
    estimate(context, features_source, features_target, correspondences);
    ssd = analyze(context, features_source, features_target, correspondences);

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
  std::string PredictCorrespondences<P, K, F>::getName() const
  {
    return "PC";
  }


  template<typename P, typename K, typename F>
  template<typename T>
  double PredictCorrespondences<P, K, F>::analyze(typename ContextType::Ptr context,
      boost::shared_ptr<pcl::PointCloud<T> > features_source,
      boost::shared_ptr<pcl::PointCloud<T> > features_target,
      boost::shared_ptr<pcl::Correspondences> correspondences)
  {
    bool write_sets = this->m_params->template get<bool>("predict_correspondences_write_sets", true);

    typename ContextType::KeypointCloud::Ptr keypoints_source = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_source");
    typename ContextType::KeypointCloud::Ptr keypoints_target = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_target");
    boost::shared_ptr<typename ContextType::Transformation> ground_truth_source = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
    boost::shared_ptr<typename ContextType::Transformation> ground_truth_target = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
    typename ContextType::KeypointCloud::Ptr keypoints_trans_source(new typename ContextType::KeypointCloud);
    typename ContextType::KeypointCloud::Ptr keypoints_trans_target(new typename ContextType::KeypointCloud);
    pcl::transformPointCloud(*keypoints_source, *keypoints_trans_source, *ground_truth_source);
    pcl::transformPointCloud(*keypoints_target, *keypoints_trans_target, *ground_truth_target);

    double ssd = 0;

    size_t row;
    if(write_sets)
    {
#ifdef PCLREF_PARALLEL
      omp_set_lock(&m_writelock);
#endif
      row = this->m_additional_logs[0]->getRowCount();
      ((DataTable*)this->m_additional_logs[0].get())->_expandBy(correspondences->size());
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

        {
          const size_t f_length = pclDataLength<T>();
          const float* f_data = pclData(f_source);
          for(int i = 0; i < f_length; ++i)
          {
            this->m_additional_logs[0]->set(row, 11 + i, f_data[i]);
          }
        }
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
  void PredictCorrespondences<P, K, F>::estimate(typename ContextType::Ptr context,
      boost::shared_ptr<pcl::PointCloud<T> > features_source,
      boost::shared_ptr<pcl::PointCloud<T> > features_target,
      boost::shared_ptr<pcl::Correspondences> correspondences)
  {
    bool reciprocal = this->m_params->template get<bool>("pc_reciprocal", true);
    bool strict = this->m_params->template get<bool>("pc_strict", true);
    double keypoint_distance_threshold = this->m_params->template get<double>("pc_keypoint_distance_threshold", 0.1);
    if(!context->exists("feature_distance_predictor"))
    {
      boost::shared_ptr<CvSVM> predictor(new CvSVM);
      std::string predictor_file = this->m_params->template get<std::string>("predictor_file");
      predictor->load(predictor_file.c_str());
      context->set("feature_distance_predictor", predictor);
    }
    boost::shared_ptr<CvSVM> classifier = context->template get<boost::shared_ptr<CvSVM> >("feature_distance_predictor");

    const size_t feature_size = pclDataLength<typename F::Type>();
    cv::Mat data;
    data.create (1, 2*feature_size, CV_32FC1);

    for(size_t i = 0; i < features_source->size(); ++i)
    {
      bool found = false;
      for(size_t j = i; j < features_target->size() && !found; ++j)
      {
        T r = features_target->points[j];
        T l = features_source->points[i];

        const float* v = pclData(l);
        for(size_t f = 0; f < feature_size; ++f)
        {
          data.at<float> (0, f) = v[f];
        }
        v = pclData(r);
        for(size_t f = 0; f < feature_size; ++f)
        {
          data.at<float> (0, f+feature_size) = v[f];
        }
        float classLR = classifier->predict(data);
        float classRL = classLR;

        if(reciprocal)
        {
          v = pclData(r);
          for(size_t f = 0; f < feature_size; ++f)
          {
            data.at<float> (0, f) = v[f];
          }
          v = pclData(l);
          for(size_t f = 0; f < feature_size; ++f)
          {
            data.at<float> (0, f+feature_size) = v[f];
          }
          classRL = classifier->predict(data);
        }

        if( strict && (classLR == 0 && classRL == 0)
            || !strict && (classLR == 0 || classRL == 0))
        {
          pcl::Correspondence corr;
          corr.index_query = i;
          corr.index_match = j;
          corr.distance = 0.1;
          correspondences->push_back(corr);
          found = true;
        }
      }
    }
  }
}

PCLREF_INSTANTIATE_PRODUCT(PredictCorrespondences,PCLREF_TYPES_PRODUCT)
#endif
