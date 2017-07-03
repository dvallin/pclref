#include <CreateKeypointIndices.h>
#include <DataTable.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  CreateKeypointIndices<P, K, F>::CreateKeypointIndices()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void CreateKeypointIndices<P, K, F>::init()
  {
    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ki_time");
    names.push_back("ki_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int CreateKeypointIndices<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::KeypointCloud::Ptr keypoints = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints");
    typename ContextType::IndexCloud::Ptr indices(new typename ContextType::IndexCloud);

    typename pcl::search::KdTree<P>::Ptr tree(new typename pcl::search::KdTree<P>);
    tree->setInputCloud(cloud);

    for (size_t i = 0; i < keypoints->size(); ++i)
    {
      std::vector<int> temp_ind(1);
      std::vector<float> temp_dist(1);
      K p = keypoints->points[i];
      P q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;
      tree->nearestKSearch(q, 1, temp_ind, temp_dist);
      indices->push_back(temp_ind[0]);
    }

    context->set("keypoint_indices", indices);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, indices->size());

    return indices->size();
  }

  template<typename P, typename K, typename F>
  std::string CreateKeypointIndices<P, K, F>::getName() const
  {
    return "KI";
  }
}

PCLREF_INSTANTIATE_PRODUCT(CreateKeypointIndices,PCLREF_TYPES_PRODUCT)
