#include <LoadGroundTruth.h>
#include <DataTable.h>
#include <pcl/io/pcd_io.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  LoadGroundTruth<P, K, F>::LoadGroundTruth()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void LoadGroundTruth<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("lg_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int LoadGroundTruth<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    boost::shared_ptr<typename ContextType::Transformation> ground_truth(new typename ContextType::Transformation);

    DataAccessor::Ptr gt_structure = context->template get<DataAccessor::Ptr>("gt_structure");
    size_t gt_col = (size_t)context->template get<int>("gt_col");
    size_t gt_line = (size_t)context->template get<int>("gt_line");
    bool gt_inline = context->template get<bool>("gt_inline");
    bool gt_matrix = context->template get<bool>("gt_from_matrix");
    gt_structure->interpretAsTransformation(gt_col, gt_line, *ground_truth, gt_inline, gt_matrix);

    context->set("ground_truth", ground_truth);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string LoadGroundTruth<P, K, F>::getName() const
  {
    return "LG";
  }
}

PCLREF_INSTANTIATE_PRODUCT(LoadGroundTruth,PCLREF_TYPES_PRODUCT)
