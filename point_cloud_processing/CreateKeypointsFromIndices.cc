#include <CreateKeypointsFromIndices.h>
#include <DataTable.h>

namespace pclref
{

  template<typename P, typename K, typename F>
  CreateKeypointsFromIndices<P, K, F>::CreateKeypointsFromIndices()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void CreateKeypointsFromIndices<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ik_time");
    names.push_back("ik_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int CreateKeypointsFromIndices<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::IndexCloud::Ptr indices = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices");

    typename ContextType::KeypointCloud::Ptr keypoints(new typename ContextType::KeypointCloud());

    for (size_t i = 0; i < indices->size(); ++i)
    {
      int idx = indices->points[i];
      P pt = cloud->points[idx];
      K kp;
      kp.x = pt.x;
      kp.y = pt.y;
      kp.z = pt.z;
      assert(isValid(kp));
      keypoints->push_back(kp);
    }
    keypoints->sensor_orientation_ = cloud->sensor_orientation_;
    keypoints->sensor_origin_ = cloud->sensor_origin_;

    context->set("keypoints", keypoints);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, keypoints->size());

    return keypoints->size();
  }

  template<typename P, typename K, typename F>
  std::string CreateKeypointsFromIndices<P, K, F>::getName() const
  {
    return "IK";
  }
}

PCLREF_INSTANTIATE_PRODUCT(CreateKeypointsFromIndices,PCLREF_TYPES_PRODUCT)
