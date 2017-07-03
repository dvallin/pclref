#include <Harris3DKeypoints.h>
#include <DataTable.h>

#include <pcl/keypoints/harris_3d.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  Harris3DKeypoints<P, K, F>::Harris3DKeypoints()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void Harris3DKeypoints<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("h3k_time");
    names.push_back("h3k_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int Harris3DKeypoints<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    typedef pcl::HarrisKeypoint3D<P, K> HarrisType;

#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::NormalCloud::Ptr normals = context->template get<typename ContextType::NormalCloud::Ptr>("normals");
    typename ContextType::KeypointCloud::Ptr keypoints(new typename ContextType::KeypointCloud);

    typename HarrisType::Ptr harris(new HarrisType);
    harris->setNonMaxSupression(this->m_params->template get<bool>("harris_non_max_supression", true));
    harris->setThreshold(this->m_params->template get<double>("harris_threshold", 1.e-5f));
    harris->setRadius(this->m_params->template get<double>("harris_radius", 0.2f));
    harris->setRefine(this->m_params->template get<bool>("harris_refine", false));
    int method = this->m_params->template get<int>("harris_method", 2);
    switch (method)
    {
    case 0:
      harris->setMethod(HarrisType::HARRIS);
      break;
    case 1:
      harris->setMethod(HarrisType::NOBLE);
      break;
    case 2:
      harris->setMethod(HarrisType::LOWE);
      break;
    case 3:
      harris->setMethod(HarrisType::TOMASI);
      break;
    case 4:
      harris->setMethod(HarrisType::CURVATURE);
      break;
    }

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

        harris->setNormals(n);
        harris->setInputCloud(c);
        harris->compute(k);
        for(size_t i = 0; i < k.points.size(); ++i)
        {
          K kp = k.points[i];
          assert(isValid(kp));
          keypoints->points.push_back(kp);
          cl->push_back((*clusters)[idx]);
        }
      }
      context->set("clusters", cl);
    }
    else
    {
      harris->setNormals(normals);
      harris->setInputCloud(cloud);
      harris->compute(*keypoints);
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
  std::string Harris3DKeypoints<P, K, F>::getName() const
  {
    return "H3K";
  }
}

PCLREF_INSTANTIATE_PRODUCT(Harris3DKeypoints,(PCLREF_POINT_TYPES)((pcl::PointXYZI))(PCLREF_FEATURE_TYPES))
