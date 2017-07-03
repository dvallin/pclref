#include <KeypointClustering.h>
#include <DataTable.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  KeypointClustering<P, K, F>::KeypointClustering()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void KeypointClustering<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("kc_time");
    names.push_back("kc_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int KeypointClustering<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::KeypointCloud::Ptr keypoints = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints");
    boost::shared_ptr<typename ContextType::Clusters> clusters(new typename ContextType::Clusters);
    clusters->resize(keypoints->size());

    typename pcl::search::KdTree<K>::Ptr search(new typename pcl::search::KdTree<K>);
    search->setInputCloud(keypoints);

    int neighbours = this->m_params->template get<int>("kc_neighbours", 1);
    double keypoint_distance_threshold = this->m_params->template get<double>("kc_keypoint_distance_threshold",
        std::numeric_limits<double>::max());

    double max_distance_sqr = keypoint_distance_threshold*keypoint_distance_threshold;
    for(size_t idx = 0; idx < cloud->size(); ++idx)
    {
      P p = cloud->points[idx];
      K k;
      k.x = p.x;
      k.y = p.y;
      k.z = p.z;

      std::vector<int> indices;
      std::vector<float> distances;
      search->nearestKSearch(k, neighbours, indices, distances);
      for(size_t n = 0; n < indices.size(); ++n)
      {
        if(distances[n] < max_distance_sqr)
          (*clusters)[indices[n]].indices.push_back(idx);
      }
    }

    context->set("clusters", clusters);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, clusters->size());

    return clusters->size();
  }

  template<typename P, typename K, typename F>
  std::string KeypointClustering<P, K, F>::getName() const
  {
    return "KC";
  }
}

PCLREF_INSTANTIATE_PRODUCT(KeypointClustering,PCLREF_TYPES_PRODUCT)
