#include <ESFFeatures.h>
#include <DataTable.h>

#include <esf_improved.h>
#include <esf_improved.hpp>

#ifndef PCLREF_LEAN
namespace pclref
{
  template<typename P, typename K>
  ESFFeatures<P, K>::ESFFeatures()
  {
    init();
  }

  template<typename P, typename K>
  void ESFFeatures<P, K>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("esf_time");
    names.push_back("esf_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K>
  int ESFFeatures<P, K>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("cloud");
    boost::shared_ptr<typename ContextType::Clusters> clusters = context->template get<boost::shared_ptr<typename ContextType::Clusters> >("clusters");
    typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);


    pcl::ESFEstimationImproved<P, pcl::ESFSignature640> esf;

    double sample_factor = this->m_params->template get<double>("esf_sample_factor", 1.0);
    int grid_size = this->m_params->template get<int>("esf_grid_size", 32);

    esf.setGridSize(grid_size);

    // Compute the features
    for(size_t idx = 0 ; idx < clusters->size(); ++idx)
    {
      typename ContextType::FeatureCloud f;
      boost::shared_ptr<std::vector<int> > indices(& ((*clusters)[idx].indices), null_deleter());
      typename ContextType::PointCloud::Ptr c(new typename ContextType::PointCloud);
      if(indices->size() < 10)
        continue;

      BOOST_FOREACH(int i, *indices)
      {
        c->points.push_back(cloud->points[i]);
      }
      esf.setInputCloud(c);
      esf.setIndices(indices);
      esf.setSampleSize(sample_factor*(unsigned int)indices->size());
      esf.compute(f);
      features->points.push_back(f.points.back());
    }

    // copy cloud origin/orientation
    features->sensor_orientation_ = cloud->sensor_orientation_;
    features->sensor_origin_ = cloud->sensor_origin_;

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
  std::string ESFFeatures<P, K>::getName() const
  {
    return "ESF";
  }
}

PCLREF_INSTANTIATE_PRODUCT(ESFFeatures,(PCLREF_POINT_TYPES)(PCLREF_KEYPOINT_TYPES))
#endif
