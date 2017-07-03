#include <GrowingRegions.h>

#ifndef PCLREF_LEAN
#include <DataTable.h>

#include <pcl/segmentation/region_growing.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  GrowingRegions<P, K, F>::GrowingRegions()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void GrowingRegions<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("gr_time");
    names.push_back("gr_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int GrowingRegions<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    typedef pcl::RegionGrowing<P, pcl::Normal>  RGType;

#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");


    int min_cluster_size = this->m_params->template get<int>("gr_min_cluster_size", 50);
    int max_cluster_size =  this->m_params->template get<int>("gr_max_cluster_size", 100000);
    int number_of_neighbours = this->m_params->template get<int>("gr_number_of_neighbours", 30);
    double smoothness_threshold = this->m_params->template get<double>("gr_smoothness_threshold", 3.0 / 180.0 * M_PI);
    double curvature_threshold = this->m_params->template get<double>("gr_curvature_threshold", 1.0);

    //
    // Compute keypoints
    //
    RGType regionGrowing;
    regionGrowing.setMinClusterSize (min_cluster_size);
    regionGrowing.setMaxClusterSize (max_cluster_size);
    regionGrowing.setNumberOfNeighbours (number_of_neighbours);
    regionGrowing.setInputCloud (cloud);
    //reg.setIndices (indices);
    regionGrowing.setInputNormals (normals);
    regionGrowing.setSmoothnessThreshold (smoothness_threshold);
    regionGrowing.setCurvatureThreshold (curvature_threshold);

    boost::shared_ptr<typename ContextType::Clusters> clusters(new typename ContextType::Clusters);
    regionGrowing.extract (*clusters);

    context->set("clusters", clusters);

    typename ContextType::KeypointCloud::Ptr keypoints(new typename ContextType::KeypointCloud);
    typename ContextType::IndexCloud::Ptr kindices(new typename ContextType::IndexCloud);
    for(size_t idx = 0 ; idx < clusters->size(); ++idx)
    {
      boost::shared_ptr<std::vector<int> > indices(& ((*clusters)[idx].indices), null_deleter());

      Eigen::Vector3f centroid = Eigen::Vector3f::Zero ();
      BOOST_FOREACH(int i, *indices)
      {
        centroid += cloud->points[i].getVector3fMap ();
      }
      centroid /= static_cast<float> (indices->size ());

      double min_diff = std::numeric_limits<double>::max();
      int k = -1;
      BOOST_FOREACH(int i, *indices)
      {
        Eigen::Vector3f point = cloud->points[i].getVector3fMap();
        double diff = (centroid - point).squaredNorm();
        if(diff < min_diff)
        {
          min_diff = diff;
          k = i;
        }
      }

      P p = cloud->points[k];
      K kp;
      kp.x = p.x;
      kp.y = p.y;
      kp.z = p.z;
      keypoints->points.push_back(kp);
      kindices->points.push_back(k);
    }
    context->set("keypoints", keypoints);
    context->set("keypoint_indices", kindices);

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
  std::string GrowingRegions<P, K, F>::getName() const
  {
    return "GR";
  }
}

PCLREF_INSTANTIATE_PRODUCT(GrowingRegions,PCLREF_TYPES_PRODUCT)
#endif
