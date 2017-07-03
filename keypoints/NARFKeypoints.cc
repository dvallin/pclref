#include <NARFKeypoints.h>
#include <DataTable.h>

#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  NARFKeypoints<P, K, F>::NARFKeypoints()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void NARFKeypoints<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("narfk_time");
    names.push_back("narfk_size");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int NARFKeypoints<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename pcl::RangeImage::Ptr range_image = context->template get<typename pcl::RangeImage::Ptr>("range_image");
    typename ContextType::IndexCloud::Ptr keypoints(new typename ContextType::IndexCloud);

    float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf (&range_image_border_extractor);
    narf.setRangeImage (range_image.get());
    narf.getParameters ().support_size = this->m_params->template get<double>("narf_keypoints_support_size", 4*model_resolution);
    narf.getParameters().max_no_of_interest_points = this->m_params->template get<int>("narf_keypoints_max_no", -1);
    narf.getParameters ().min_distance_between_interest_points = this->m_params->template get<double>("narf_keypoints_min_distance", 2*model_resolution);
    narf.getParameters ().optimal_distance_to_high_surface_change = this->m_params->template get<double>("narf_keypoints_optimal_distance_to_high_surface_change", 2*model_resolution);
    narf.getParameters().min_interest_value = this->m_params->template get<double>("narf_keypoints_min_interest_value", 0.45f);
    narf.getParameters().min_surface_change_score = this->m_params->template get<double>("narf_keypoints_min_surface_change_score", 0.2f);
    narf.getParameters().optimal_range_image_patch_size = this->m_params->template get<int>("optimal_range_image_patch_size", 10);
    narf.getParameters().distance_for_additional_points = this->m_params->template get<float>("narf_keypoints_distance_for_additional_points", 0.f);
    narf.getParameters().add_points_on_straight_edges = this->m_params->template get<bool>("narf_keypoints_add_points_on_straight_edges", false);
    narf.getParameters().do_non_maximum_suppression = this->m_params->template get<bool>("narf_keypoints_do_non_maximum_suppression", true);
    narf.getParameters().no_of_polynomial_approximations_per_point = this->m_params->template get<int>("narf_keypoints_no_of_polynomial_approximations_per_point", 0);
    narf.getParameters().max_no_of_threads = this->m_params->template get<int>("narf_keypoints_max_no_of_threads", 1);
    narf.getParameters().use_recursive_scale_reduction = this->m_params->template get<bool>("narf_keypoints_use_recursive_scale_reduction", false);
    narf.getParameters().calculate_sparse_interest_image = this->m_params->template get<bool>("narf_keypoints_calculate_sparse_interest_image", true);
    narf.compute (*keypoints);

    context->set("keypoint_indices", keypoints);

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
  std::string NARFKeypoints<P, K, F>::getName() const
  {
    return "NARFK";
  }
}

PCLREF_INSTANTIATE_PRODUCT(NARFKeypoints,PCLREF_TYPES_PRODUCT)
