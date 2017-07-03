#include <FeaturePCA.h>
#include <DataTable.h>

#include <general_pca.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  FeaturePCA<P, K, F>::FeaturePCA()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void FeaturePCA<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("fpca_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void FeaturePCA<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();

    bool write = this->m_params->template get<bool>("pca_write", false);

    if(write)
    {
      size_t dataLength = pclref::pclDataLength<typename F::Type>();
      std::vector<DataTable::columnType> types;
      std::vector<std::string> names;
      for(int i = 0; i < dataLength; ++i)
      {
        types.push_back(DataTable::FLOAT);
        names.push_back("c" + precision_cast<int>(i));
      }

      typename DataTable::Ptr log = typename DataTable::Ptr(new DataTable(types, names));
      log->setName("pca_log");
      this->m_additional_logs.push_back(log);
    }
  }

  template<typename P, typename K, typename F>
  int FeaturePCA<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);
    generateInputFeatureSpace(context, features);

    projectFeatureSpaces(context, features);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string FeaturePCA<P, K, F>::getName() const
  {
    return "FPCA";
  }

  template<typename P, typename K, typename F>
  void FeaturePCA<P, K, F>::generateInputFeatureSpace(typename ContextType::Ptr context,
      typename ContextType::FeatureCloud::Ptr features)
  {
    // this step is run accumulated. this means we have a context of contexts containing feature spaces we shall use
    if(context->exists("accu") && context->template get<bool>("accu"))
    {
      typename ContextType::Ptr accu = context->template get<typename ContextType::Ptr>("kfContexts");

      typename ContextType::iterator iterI = accu->begin();
      typename ContextType::iterator iend = accu->end();
      for(; iterI != iend; ++iterI)
      {
        typename ContextType::Ptr sourceContext = boost::any_cast<typename ContextType::Ptr>(*iterI);
        typename ContextType::FeatureCloud::Ptr f = sourceContext->template get<typename ContextType::FeatureCloud::Ptr>("features");
        *features += *f;
      }
    }
    else
    {
      typename ContextType::FeatureCloud::Ptr f = context->template get<typename ContextType::FeatureCloud::Ptr>("features");

      *features = *f;
    }
  }

  template<typename P, typename K, typename F>
  void FeaturePCA<P, K, F>::projectFeatureSpaces(typename ContextType::Ptr context, typename ContextType::FeatureCloud::Ptr features)
  {
    boost::shared_ptr<pcl::GeneralPCA<typename F::Type> > pca;
    if(context->exists("pca_stats"))
    {
      DataAccessor::Ptr pca_stats = context->template get<DataAccessor::Ptr>("pca_stats");
      size_t dataLength = pclref::pclDataLength<typename F::Type>();
      Eigen::VectorXd eigenVal(dataLength);
      Eigen::MatrixXd eigenVec(dataLength, dataLength);
      Eigen::VectorXd mean(dataLength);

      pca_stats->interpretAsEigenVector(2, 0, eigenVal);
      pca_stats->interpretAsEigenMatrix(2, 1, eigenVec, false);
      pca_stats->interpretAsEigenVector(2, dataLength+1, mean);

      pca.reset(new pcl::GeneralPCA<typename F::Type>(eigenVec,mean, eigenVal));
    }
    else
    {
      pca.reset(new pcl::GeneralPCA<typename F::Type>(true));
      pca->setInputCloud(features);
    }

    // this step is run accumulated. so there is nothing to be projected (we will just use all data
    // to generate a transformed space). So there is just some pca information to be written.
    if(context->exists("accu") && context->template get<bool>("accu"))
    {
      Eigen::VectorXd eigenVal = pca->getEigenValues();
      Eigen::MatrixXd eigenVec = pca->getEigenVectors();
      Eigen::VectorXd mean = pca->getMean();

      bool write = this->m_params->template get<bool>("pca_write", false);

      if(write)
      {
        size_t dataLength = pclref::pclDataLength<typename F::Type>();
        size_t row = this->m_additional_logs[0]->getRowCount();
        this->m_additional_logs[0]->writeEigenVector(0, row, eigenVal);
        this->m_additional_logs[0]->writeEigenMatrix(0, row+1, eigenVec, false);
        this->m_additional_logs[0]->writeEigenVector(0, row+dataLength+1, mean);
      }

      context->add("pca_eigen_values", eigenVal);
      context->add("pca_eigen_vectors", eigenVec);
      context->add("pca_mean", mean);
    }
    else
    {
      bool whitened = this->m_params->template get<bool>("pca_whitened", true);
      bool use_zca = this->m_params->template get<bool>("pca_use_zca", false);
      int dimensions = this->m_params->template get<int>("pca_dimensions", 0);
      if(dimensions > 0) pca->setProjectionDimension(dimensions);

      typename ContextType::FeatureCloud::Ptr features = context->template get<typename ContextType::FeatureCloud::Ptr>("features");
      typename ContextType::FeatureCloud::Ptr features_projected(new typename ContextType::FeatureCloud);
      pca->project(*features, *features_projected, whitened, use_zca);

      context->set("features", features_projected);
    }
  }
}

PCLREF_INSTANTIATE_PRODUCT(FeaturePCA,PCLREF_TYPES_PRODUCT)
