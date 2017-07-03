#include <KeypointNeighborhood.h>
#include <DataTable.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  KeypointNeighborhood<P, K, F>::KeypointNeighborhood()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void KeypointNeighborhood<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("kn_time");
    names.push_back("kn_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int KeypointNeighborhood<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
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

    typename pcl::search::KdTree<P>::Ptr search(new typename pcl::search::KdTree<P>);
    search->setInputCloud(cloud);

    double radius = this->m_params->template get<double>("kn_radius", 0.4);
    bool extract_scale = this->m_params->template get<double>("kn_extract_scale", true);

    GetScale<K> getScale(radius, extract_scale);
    for(size_t idx = 0; idx < keypoints->size(); ++idx)
    {
      K k = keypoints->points[idx];
      P p;
      p.x = k.x;
      p.y = k.y;
      p.z = k.z;

      std::vector<int> indices;
      std::vector<float> distances;

      float scale = getScale(k);
      search->radiusSearchT(p, scale, indices, distances);

      (*clusters)[idx].indices.assign(indices.begin(), indices.end());
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
  std::string KeypointNeighborhood<P, K, F>::getName() const
  {
    return "KN";
  }
}

PCLREF_INSTANTIATE_PRODUCT(KeypointNeighborhood,PCLREF_TYPES_PRODUCT)
