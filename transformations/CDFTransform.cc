#include <CDFTransform.h>
#include <DataTable.h>
#include <CDF.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  CDFTransform<P, K, F>::CDFTransform()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void CDFTransform<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("cdf_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void CDFTransform<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();

    bool write = this->m_params->template get<bool>("cdf_write", false);

    if(write)
    {
      size_t dataLength = pclDataLength<typename F::Type>();
      std::vector<DataTable::columnType> types;
      std::vector<std::string> names;
      for(int i = 0; i < dataLength; ++i)
      {
        types.push_back(DataTable::FLOAT);
        names.push_back("c" + precision_cast<int>(i));
      }

      typename DataTable::Ptr log = typename DataTable::Ptr(new DataTable(types, names));
      log->setName("cdf_log");
      this->m_additional_logs.push_back(log);
    }
  }

  template<typename P, typename K, typename F>
  int CDFTransform<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
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
  std::string CDFTransform<P, K, F>::getName() const
  {
    return "CDF";
  }

  template<typename P, typename K, typename F>
  void CDFTransform<P, K, F>::generateInputFeatureSpace(typename ContextType::Ptr context,
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
  void CDFTransform<P, K, F>::projectFeatureSpaces(typename ContextType::Ptr context, typename ContextType::FeatureCloud::Ptr features)
  {
    boost::shared_ptr<CDF<typename F::Type> > cdf;
    if(context->exists("cdf_stats"))
    {
      DataAccessor::Ptr cdf_stats = context->template get<DataAccessor::Ptr>("cdf_stats");
      Eigen::MatrixXf cdf_matrix(cdf_stats->getRowCount(), pclDataLength<typename F::Type>());
      cdf_stats->interpretAsEigenMatrix(2, 0, cdf_matrix, false);

      cdf.reset(new CDF<typename F::Type>(cdf_matrix));
    }
    else
    {
      int samples = this->m_params->template get<int>("cdf_samples", 20);
      cdf.reset(new CDF<typename F::Type>(samples));
      cdf->setInputCloud(features);
    }

    // this step is run accumulated. so there is nothing to be projected (we will just use all data
    // to generate a transformed space). So there is just some cdf information to be written.
    if(context->exists("accu") && context->template get<bool>("accu"))
    {
      Eigen::MatrixXf cdf_matrix = cdf->getMatrix();

      bool write = this->m_params->template get<bool>("cdf_write", false);

      if(write)
      {
        size_t row = this->m_additional_logs[0]->getRowCount();
        this->m_additional_logs[0]->writeEigenMatrix(0, row, cdf_matrix, false);
      }

      context->add("cdf_matrix", cdf_matrix);
    }
    else
    {
      typename ContextType::FeatureCloud::Ptr features = context->template get<typename ContextType::FeatureCloud::Ptr>("features");
      typename ContextType::FeatureCloud::Ptr features_projected(new typename ContextType::FeatureCloud);
      cdf->project(*features, *features_projected);
      context->set("features", features_projected);
    }
  }
}

PCLREF_INSTANTIATE_PRODUCT(CDFTransform,PCLREF_TYPES_PRODUCT)
