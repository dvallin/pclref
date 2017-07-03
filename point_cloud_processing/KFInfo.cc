#include <KFInfo.h>
#include <DataTable.h>

#include <pcl/io/pcd_io.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  KFInfo<P, K, F>::KFInfo()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void KFInfo<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("id");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int KFInfo<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    this->m_logger->set(run, 0, context->template get<identifier>("id"));

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string KFInfo<P, K, F>::getName() const
  {
    return "KFINFO";
  }
}

PCLREF_INSTANTIATE_PRODUCT(KFInfo,PCLREF_TYPES_PRODUCT)
