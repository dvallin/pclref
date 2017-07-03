#include <LoadCloud.h>
#include <DataTable.h>

#include <pcl/io/pcd_io.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  LoadCloud<P, K, F>::LoadCloud()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void LoadCloud<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::ENUM);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("lc_time");
    names.push_back("lc_id");
    names.push_back("lc_file");
    names.push_back("lc_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int LoadCloud<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud(new typename ContextType::PointCloud);

    typename ContextType::Datum datum = context->getDatum("load_cloud_structure");
    std::string source_string = "unkown";
    if(datum.type() == typeid(std::string))
    {
      source_string = boost::any_cast<std::string>(datum);
      process(source_string, cloud);
    }
    else if(datum.type() == typeid(DataAccessor::Ptr))
    {
      source_string = "data_accessor";
      process(boost::any_cast<DataAccessor::Ptr>(datum), cloud);
    }

    cloud->sensor_origin_[3] = 1.0;
    context->set("cloud", cloud);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, context->template get<identifier>("id"));
    this->m_logger->set(run, 2, source_string);
    this->m_logger->set(run, 3, cloud->size());

    return cloud->size();
  }

  template<typename P, typename K, typename F>
  std::string LoadCloud<P, K, F>::getName() const
  {
    return "LC";
  }

  template<typename P, typename K, typename F>
  void LoadCloud<P, K, F>::process(const std::string& filename, typename ContextType::PointCloud::Ptr cloud)
  {
    if (pcl::io::loadPCDFile<P> (filename, *cloud) == -1)   //* load the file
    {
      PCL_ERROR ("LoadCloud: couldn't read file\n");
    }
  }

  template<typename P, typename K, typename F>
  void LoadCloud<P, K, F>::process(const DataAccessor::Ptr& acc, typename ContextType::PointCloud::Ptr cloud)
  {
    size_t x_column = (size_t)this->m_params->template get<int>("cloud_x_column", 0);
    size_t y_column = (size_t)this->m_params->template get<int>("cloud_y_column", 1);
    size_t z_column = (size_t)this->m_params->template get<int>("cloud_z_column", 2);
    std::vector<P> points;
    acc->template interpretAsXYZ<P>(x_column, y_column, z_column, points);

    BOOST_FOREACH(P& p, points)
    {
      assert(isValid(p));
      cloud->push_back(p);
    }
  }
}

PCLREF_INSTANTIATE_PRODUCT(LoadCloud,PCLREF_TYPES_PRODUCT)
