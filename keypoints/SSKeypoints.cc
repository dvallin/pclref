#include <SSKeypoints.h>

#ifndef PCLREF_LEAN
#include <DataTable.h>

#include <pcl/keypoints/smoothed_surfaces_keypoint.h>

namespace pclref
{
  template<typename P, typename F>
  SSKeypoints<P, F>::SSKeypoints()
  {
    init();
  }

  template<typename P, typename F>
  void SSKeypoints<P, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ss_time");
    names.push_back("ss_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename F>
  int SSKeypoints<P, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    typedef pcl::SmoothedSurfacesKeypoint<P, pcl::Normal> SSType;

#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");
    typename ContextType::KeypointCloud::Ptr keypoints(new typename ContextType::KeypointCloud);

    typename SSType::Ptr ss(new SSType);
    ss->setNeighborhoodConstant(this->m_params->template get<double>("ss_neighbourhood_constant", 0.4));
    ss->setInputScale (this->m_params->template get<double>("ss_input_scale", 1.0));

    if(context->exists("clusters"))
    {
      boost::shared_ptr<typename ContextType::Clusters> clusters = context->template get<boost::shared_ptr<typename ContextType::Clusters> >("clusters");
      boost::shared_ptr<typename ContextType::Clusters> cl(new typename ContextType::Clusters);
      for(size_t idx = 0 ; idx < clusters->size(); ++idx)
      {
        typename ContextType::KeypointCloud k;
        boost::shared_ptr<std::vector<int> > indices(& ((*clusters)[idx].indices), null_deleter());
        typename ContextType::PointCloud::Ptr c(new typename ContextType::PointCloud);
        typename ContextType::NormalCloud::Ptr n(new typename ContextType::NormalCloud);
        BOOST_FOREACH(int i, *indices)
        {
          c->points.push_back(cloud->points[i]);
          n->points.push_back(normals->points[i]);
        }

        ss->setInputNormals(n);
        ss->setInputCloud(c);
        ss->compute(k);
        for(size_t i = 0; i < k.points.size(); ++i)
        {
          P kp = k.points[i];
          assert(isValid(kp));
          keypoints->points.push_back(kp);
          cl->push_back((*clusters)[idx]);
        }
      }
      context->set("clusters", cl);
    }
    else
    {
      ss->setInputNormals(normals);
      ss->setInputCloud(cloud);
      ss->compute(*keypoints);
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

  template<typename P, typename F>
  std::string SSKeypoints<P, F>::getName() const
  {
    return "SS";
  }
}

PCLREF_INSTANTIATE_PRODUCT(SSKeypoints,(PCLREF_POINT_TYPES)(PCLREF_FEATURE_TYPES))
#endif
