#include <FPFHFeatures.h>
#include <DataTable.h>

// in all cases i've seen so far without omp its faster.
#ifdef FPFH_USE_OMP
#include <pcl/features/fpfh_omp.h>
#else
#include <pcl/features/fpfh.h>
#endif

namespace pclref
{
  template<typename P, typename K>
  FPFHFeatures<P, K>::FPFHFeatures()
  {
    init();
  }

  template<typename P, typename K>
  void FPFHFeatures<P, K>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("fpfhf_time");
    names.push_back("fpfhf_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K>
  int FPFHFeatures<P, K>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");
    typename ContextType::IndexCloud::Ptr keypoint_indices = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices");
    typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);

    if(keypoint_indices->size() > 0)
    {
      boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
      indices->reserve(keypoint_indices->size());
      BOOST_FOREACH(int index, *keypoint_indices)
      {
        indices->push_back(index);
      }

#ifdef FPFH_USE_OMP
      pcl::FPFHEstimationOMP<P, pcl::Normal, pcl::FPFHSignature33> fpfh;
#else
      pcl::FPFHEstimation<P, pcl::Normal, pcl::FPFHSignature33> fpfh;
#endif

      fpfh.setInputNormals(normals);
      fpfh.setIndices(indices);
      fpfh.setInputCloud(cloud);
      if(!this->m_params->exists("fpfh_radius_search"))
        fpfh.setKSearch(this->m_params->template get<int>("fpfh_k_search", 50));
      else
        fpfh.setRadiusSearch(this->m_params->template get<double>("fpfh_radius_search", 0.8));
      // Compute the features
      fpfh.compute(*features);
    }

    // copy cloud origin/orientation
    //features->sensor_orientation_ = cloud->sensor_orientation_;
    //features->sensor_origin_ = cloud->sensor_origin_;

    context->set("features", features);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, features->size());

    return features->size();
  }

  template<typename P, typename K>
  std::string FPFHFeatures<P, K>::getName() const
  {
    return "FPFH";
  }
}

PCLREF_INSTANTIATE_PRODUCT(FPFHFeatures,(PCLREF_POINT_TYPES)(PCLREF_KEYPOINT_TYPES))
