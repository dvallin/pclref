#include <RandomKeypoints.h>

#ifndef PCLREF_LEAN
#include <DataTable.h>

#include <Arrays.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  RandomKeypoints<P, K, F>::RandomKeypoints()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void RandomKeypoints<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("rk_time");
    names.push_back("rk_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int RandomKeypoints<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::KeypointCloud::Ptr keypoints(new typename ContextType::KeypointCloud);
    typename ContextType::IndexCloud::Ptr keypoint_indices(new typename ContextType::IndexCloud);

    std::vector<size_t> indices;
    Arrays::pick(0, cloud->points.size(), 100, indices);
    for(size_t i = 0; i < indices.size(); ++i)
    {
      int idx = indices[i];
      keypoint_indices->points.push_back(idx);
      P p = cloud->points[idx];
      K k;
      k.x = p.x;
      k.y = p.y;
      k.z = p.z;
      keypoints->points.push_back(k);
    }

    context->set("keypoints", keypoints);
    context->set("keypoint_indices", keypoint_indices);

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
  std::string RandomKeypoints<P, K, F>::getName() const
  {
    return "RK";
  }
}

PCLREF_INSTANTIATE_PRODUCT(RandomKeypoints,PCLREF_TYPES_PRODUCT)
#endif
