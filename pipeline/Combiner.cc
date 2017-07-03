#include <Combiner.h>
#include <DataMerge.h>
#include <DataTable.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  Combiner<P, K, F>::Combiner()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void Combiner<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("c_time");
    m_logger_internal = boost::shared_ptr<DataTable>(new DataTable(types, names));

    this->m_logger = boost::shared_ptr<DataMerge>(new DataMerge(false));
    static_cast<DataMerge*>(this->m_logger.get())->addAccessor(m_logger_internal);

    BOOST_FOREACH(Ptr step, m_steps)
    {
      step->init();
      static_cast<DataMerge*>(this->m_logger.get())->addAccessor(step->getLogger());
    }
  }

  template<typename P, typename K, typename F>
  int Combiner<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    const clock_t t_outer = clock();
#ifdef PCLREF_PARALLEL
    double w_outer = omp_get_wtime();
#endif

    this->m_additional_logs.clear();

    const size_t c = context->size();

    BOOST_FOREACH(Ptr step, m_steps)
    {
      step->setParameterContext(this->m_params);
      step->prepareAdditionalLogs();
      this->m_additional_logs.insert(this->m_additional_logs.end(),
                                     step->getAdditionalLogs().begin(),
                                     step->getAdditionalLogs().end());
    }


#ifdef PCLREF_PARALLEL
    omp_set_num_threads(this->m_params->template get<int>("threads", 1));
    #pragma omp parallel for
#endif
    for(size_t i = 0; i < c; ++i)
    {
#ifdef PCLREF_PARALLEL
      double t0 = omp_get_wtime();
#else
      const clock_t t0 = clock();
#endif

      typename ContextType::Ptr context_inner = boost::any_cast<typename ContextType::Ptr>(*(context->operator [](i)));

      // reset for next run, i.e. set inputs
      BOOST_FOREACH(Ptr step, m_steps)
      {
        int success = step->compute(i, context_inner); // do nothing if unsuccessful for now
      }

#ifdef PCLREF_PARALLEL
      double ms = (double(omp_get_wtime() - t0)*1000);
#else
      double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

      m_logger_internal->set(i, 0, ms);
    }

    double ms_cpu = (double(clock() - t_outer)*1000) / CLOCKS_PER_SEC;
#ifdef PCLREF_PARALLEL
    double ms_wall = (omp_get_wtime() - w_outer)*1000;
    if(this->m_params->template get<bool>("verbose", false))
    {
      std::cout << "cpu time " << ms_cpu <<
                ", wall time " << ms_wall <<
                " (usage " << ms_cpu / ms_wall << ")" << std::endl;
    }
#else
    if(this->m_params->template get<bool>("verbose", false))
      std::cout << "cpu time " << ms_cpu << std::endl;
#endif

    return c;
  }

  template<typename P, typename K, typename F>
  std::string Combiner<P, K, F>::getName() const
  {
    std::string name = "C";
    BOOST_FOREACH(Ptr step, m_steps)
    {
      name += std::string("_");
      name += step->getName();
    }
    return name;
  }

  template<typename P, typename K, typename F>
  void Combiner<P, K, F>::addStep(Ptr step)
  {
    m_steps.push_back(step);
    static_cast<DataMerge*>(this->m_logger.get())->addAccessor(step->getLogger());
  }
}

PCLREF_INSTANTIATE_PRODUCT(Combiner,PCLREF_TYPES_PRODUCT)
