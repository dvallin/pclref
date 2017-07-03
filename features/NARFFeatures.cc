#include <NARFFeatures.h>
#include <DataTable.h>

#include <pcl/features/narf_descriptor.h>

namespace pclref
{
  template<typename P, typename K>
  NARFFeatures<P, K>::NARFFeatures()
  {
    init();
  }

  template<typename P, typename K>
  void NARFFeatures<P, K>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("narff_time");
    names.push_back("narff_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K>
  int NARFFeatures<P, K>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::RangeImage::Ptr range_image = context->template get<typename ContextType::RangeImage::Ptr>("range_image");
    typename ContextType::IndexCloud::Ptr keypoint_indices = context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices");
    typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);

    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
    indices->reserve(keypoint_indices->size());
    BOOST_FOREACH(int index, *keypoint_indices)
    {
      indices->push_back(index);
    }

    pcl::NarfDescriptor narf (range_image.get(), indices.get());
    narf.getParameters().support_size = this->m_params->template get<double>("narf_features_support_size", 0.8);
    narf.getParameters().rotation_invariant = this->m_params->template get<bool>("narf_features_rotation_invariant", true);
    narf.compute(*features);

    // rebuild KeypointCloud using new found points
    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr>("remapped_cloud", "cloud");
    typename ContextType::KeypointCloud::Ptr keypoints(new typename ContextType::KeypointCloud);
    typename ContextType::IndexCloud::Ptr new_indices(new typename ContextType::IndexCloud);
    typename pcl::search::KdTree<P>::Ptr tree(new typename pcl::search::KdTree<P>);
    tree->setInputCloud(cloud);
    for(size_t idx = 0; idx < features->size(); ++idx)
    {
      std::vector<int> temp_ind(1);
      std::vector<float> temp_dist(1);
      pcl::Narf36 feature = features->points[idx];
      assert(isValid(feature));

      K keypoint;
      keypoint.x = feature.x;
      keypoint.y = feature.y;
      keypoint.z = feature.z;
      keypoints->points.push_back(keypoint);

      P point;
      point.x = feature.x;
      point.y = feature.y;
      point.z = feature.z;
      tree->nearestKSearch(point, 1, temp_ind, temp_dist);
      new_indices->push_back(temp_ind[0]);
    }
    context->set("keypoint_indices", new_indices);
    context->set("keypoints", keypoints);

    context->set("features", features);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, keypoint_indices->size());

    return features->size();
  }

  template<typename P, typename K>
  std::string NARFFeatures<P, K>::getName() const
  {
    return "NARFF";
  }
}

PCLREF_INSTANTIATE_PRODUCT(NARFFeatures,(PCLREF_POINT_TYPES)(PCLREF_KEYPOINT_TYPES))
