#include <LRTTransform.h>
#include <DataTable.h>
#include <LRT.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  LRTTransform<P, K, F>::LRTTransform()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void LRTTransform<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("lrt_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void LRTTransform<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();

    bool write = this->m_params->template get<bool>("lrt_write", false);

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
      log->setName("lrt_log");
      this->m_additional_logs.push_back(log);
    }
  }

  template<typename P, typename K, typename F>
  int LRTTransform<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::FeatureCloud::Ptr matches(new typename ContextType::FeatureCloud);
    typename ContextType::FeatureCloud::Ptr unmatches(new typename ContextType::FeatureCloud);
    generateInputFeatureSpace(context, matches, unmatches);
    projectFeatureSpaces(context, matches, unmatches);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string LRTTransform<P, K, F>::getName() const
  {
    return "LRT";
  }

  template<typename P, typename K, typename F>
  void LRTTransform<P, K, F>::generateInputFeatureSpace(typename ContextType::Ptr context,
      typename ContextType::FeatureCloud::Ptr matches,
      typename ContextType::FeatureCloud::Ptr unmatches)
  {
    // this step is run accumulated. this means we have a context of contexts containing feature spaces we shall use
    if(context->exists("accu") && context->template get<bool>("accu"))
    {
      typename ContextType::Ptr accu = context->template get<typename ContextType::Ptr>("ceContexts");

      typename ContextType::iterator iterI = accu->begin();
      typename ContextType::iterator iend = accu->end();
      for(; iterI != iend; ++iterI)
      {
        typename ContextType::Ptr sourceContext = boost::any_cast<typename ContextType::Ptr>(*iterI);
        typename ContextType::FeatureCloud::Ptr m = sourceContext->template get<typename ContextType::FeatureCloud::Ptr>("match_feature_diffs");
        typename ContextType::FeatureCloud::Ptr u = sourceContext->template get<typename ContextType::FeatureCloud::Ptr>("unmatch_feature_diffs");
        *matches += *m;
        *unmatches += *u;
      }
    }
  }

  template<typename P, typename K, typename F>
  void LRTTransform<P, K, F>::projectFeatureSpaces(typename ContextType::Ptr context,
      typename ContextType::FeatureCloud::Ptr matches,
      typename ContextType::FeatureCloud::Ptr unmatches)
  {
    boost::shared_ptr<LRT<typename F::Type> > lrt;
    if(context->exists("lrt_stats"))
    {
      DataAccessor::Ptr lrt_stats = context->template get<DataAccessor::Ptr>("lrt_stats");
      Eigen::MatrixXd lrt_matrix(lrt_stats->getRowCount(), pclDataLength<typename F::Type>());
      lrt_stats->interpretAsEigenMatrix(2, 0, lrt_matrix, false);

      lrt.reset(new LRT<typename F::Type>(lrt_matrix));
    }
    else
    {
      lrt.reset(new LRT<typename F::Type>());
      lrt->setMatchDiffs(matches);
      lrt->setUnmatchDiffs(unmatches);
    }

    // this step is run accumulated. so there is nothing to be projected (we will just use all data
    // to generate a transformed space). So there is just some pca information to be written.
    if(context->exists("accu") && context->template get<bool>("accu"))
    {
      Eigen::MatrixXd lrt_matrix = lrt->getTransform();

      bool write = this->m_params->template get<bool>("lrt_write", false);

      if(write)
      {
        size_t row = this->m_additional_logs[0]->getRowCount();
        this->m_additional_logs[0]->writeEigenMatrix(0, row, lrt_matrix, false);
      }

      context->add("lrt_matrix", lrt_matrix);
    }
    else
    {
      typename ContextType::FeatureCloud::Ptr features = context->template get<typename ContextType::FeatureCloud::Ptr>("features");
      typename ContextType::FeatureCloud::Ptr features_projected(new typename ContextType::FeatureCloud);
      lrt->project(*features, *features_projected);
      context->set("features", features_projected);
    }
  }
}

PCLREF_INSTANTIATE_PRODUCT(LRTTransform,PCLREF_TYPES_PRODUCT)
