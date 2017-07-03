#include <RemoveRemap.h>
#include <DataTable.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  RemoveRemap<P, K, F>::RemoveRemap()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void RemoveRemap<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("rr_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int RemoveRemap<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    context->remove("remapped_cloud");

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string RemoveRemap<P, K, F>::getName() const
  {
    return "RR";
  }
}

PCLREF_INSTANTIATE_PRODUCT(RemoveRemap,PCLREF_TYPES_PRODUCT)
