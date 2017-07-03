#include <TInfo.h>
#include <DataTable.h>

#include <pcl/io/pcd_io.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  TInfo<P, K, F>::TInfo()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void TInfo<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("source_id");
    names.push_back("target_id");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int TInfo<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    this->m_logger->set(run, 0, context->template get<identifier>("source_id"));
    this->m_logger->set(run, 1, context->template get<identifier>("target_id"));

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string TInfo<P, K, F>::getName() const
  {
    return "TINFO";
  }
}


PCLREF_INSTANTIATE_PRODUCT(TInfo,PCLREF_TYPES_PRODUCT)
