#include <RANSAC.h>
#include <DataTable.h>

#include <generic_ransac_correspondence_rejector.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/rmsac.h>

#include <sac_model_general_registration.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  Ransac<P, K, F>::Ransac()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void Ransac<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("ransac_time");
    names.push_back("ransac_size");
    names.push_back("ransac_residual");
    names.push_back("ransac_all_residual");
    names.push_back("ransac_translation_error");
    names.push_back("ransac_rotation_error");
    names.push_back("ransac_score");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int Ransac<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::KeypointCloud::Ptr keypoints_source = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_source");
    typename ContextType::KeypointCloud::Ptr keypoints_target = context->template get<typename ContextType::KeypointCloud::Ptr>("keypoints_target");
    typename boost::shared_ptr<pcl::Correspondences> correspondences = context->template get<typename boost::shared_ptr<pcl::Correspondences> > ("correspondences");
    boost::shared_ptr<pcl::Correspondences> correspondences_inlier(new pcl::Correspondences);

    double ransac_inlier_threshold = this->m_params->template get<double>("ransac_inlier_threshold", 1.5);
    bool refine_model = this->m_params->template get<bool>("ransac_refine_model", false);
    bool general_model = this->m_params->template get<bool>("ransac_general_model", false);
    int max_iterations = this->m_params->template get<int>("ransac_max_iterations", 1000);
    int ransac_method = this->m_params->template get<int>("ransac_method", pcl::SAC_MSAC);
    double probability = this->m_params->template get<double>("ransac_probability", 0.99);

    pcl::registration::GenericRansacCorrespondenceRejector<K> ransac;
    ransac.setRefineModel(refine_model);
    ransac.setInputTarget(keypoints_target);
    ransac.setInputSource(keypoints_source);
    ransac.setInlierThreshold(ransac_inlier_threshold);
    ransac.setMaximumIterations(max_iterations);
    ransac.setProbability(probability);

    switch(ransac_method)
    {
    case pcl::SAC_RANSAC:
    {
      typedef pcl::RandomSampleConsensus<K> RansacMethod;
      if(general_model)
      {
        typedef pcl::SampleConsensusModelGeneralRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
      else
      {
        typedef pcl::SampleConsensusModelRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
    }
    break;
    case pcl::SAC_LMEDS:
    {
      typedef pcl::LeastMedianSquares<K> RansacMethod;
      if(general_model)
      {
        typedef pcl::SampleConsensusModelGeneralRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
      else
      {
        typedef pcl::SampleConsensusModelRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
    }
    break;
    case pcl::SAC_MSAC:
    {
      typedef pcl::MEstimatorSampleConsensus<K> RansacMethod;
      if(general_model)
      {
        typedef pcl::SampleConsensusModelGeneralRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
      else
      {
        typedef pcl::SampleConsensusModelRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
    }
    break;
    case pcl::SAC_RRANSAC:
    {
      typedef pcl::RandomizedRandomSampleConsensus<K> RansacMethod;
      ransac.setSamplePercentage(this->m_params->template get<double>("ransac_sample_percentage", 10.0));
      if(general_model)
      {
        typedef pcl::SampleConsensusModelGeneralRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
      else
      {
        typedef pcl::SampleConsensusModelRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
    }
    break;
    case pcl::SAC_RMSAC:
    {
      typedef pcl::RandomizedMEstimatorSampleConsensus<K> RansacMethod;
      ransac.setSamplePercentage(this->m_params->template get<double>("ransac_sample_percentage", 10.0));
      if(general_model)
      {
        typedef pcl::SampleConsensusModelGeneralRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
      else
      {
        typedef pcl::SampleConsensusModelRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
    }
    break;
    case pcl::SAC_MLESAC:
    {
      typedef pcl::MaximumLikelihoodSampleConsensus<K> RansacMethod;
      ransac.setEmIterations(this->m_params->template get<int>("ransac_em_iterations", 3));
      if(general_model)
      {
        typedef pcl::SampleConsensusModelGeneralRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
      else
      {
        typedef pcl::SampleConsensusModelRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
    }
    break;
    case pcl::SAC_PROSAC:
    {
      typedef pcl::ProgressiveSampleConsensus<K> RansacMethod;
      if(general_model)
      {
        typedef pcl::SampleConsensusModelGeneralRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
      else
      {
        typedef pcl::SampleConsensusModelRegistration<K> RansacModel;
        ransac.template getRemainingCorrespondences<RansacMethod, RansacModel>(*correspondences, *correspondences_inlier, general_model);
      }
    }
    break;
    }


    Eigen::Affine3f keypoint_transformation = Eigen::Affine3f::Identity();
    float residual = -1.f;
    float all_residual = -1.f;

    bool valid_transform = true;

    // just for savekeeping :I
    context->set("correspondences_unfiltered", correspondences);
    if (!correspondences_inlier->empty())
    {
      keypoint_transformation = Eigen::Affine3f(ransac.getBestTransformation());

      // Check whether the transformation found was a valid one. TODO: I actually should also check the angles...
      Eigen::Vector3f translation = keypoint_transformation.translation();
      if (translation[0] == 0 && translation[1] == 0 && translation[2] == 0 ||
          std::isnan(translation[0]) || std::isnan(translation[1]) || std::isnan(translation[2]))
      {
        valid_transform = false;
        keypoint_transformation = Eigen::Affine3f::Identity();
      }

      // calc residuals
      if(this->m_params->get("ransac_calc_residuals", true))
      {
        float ssd = 0;
        typename ContextType::KeypointCloud::Ptr keypoints_trans_target(new typename ContextType::KeypointCloud);

        pcl::transformPointCloud(*keypoints_target, *keypoints_trans_target, keypoint_transformation);
        assert(keypoints_target->size() == keypoints_trans_target->size());

        for (size_t i = 0; i < correspondences_inlier->size(); ++i)
        {
          size_t query_idx = (*correspondences_inlier)[i].index_query;
          size_t match_idx = (*correspondences_inlier)[i].index_match;
          K k_source = keypoints_source->points[query_idx];
          K k_target = keypoints_trans_target->points[match_idx];
          ssd += distance(k_source, k_target, pcl::L2_SQR);
        }
        residual = ssd;

        ssd = 0;
        for (size_t i = 0; i < correspondences->size(); ++i)
        {
          size_t query_idx = (*correspondences)[i].index_query;
          size_t match_idx = (*correspondences)[i].index_match;
          K k_source = keypoints_source->points[query_idx];
          K k_target = keypoints_trans_target->points[match_idx];
          ssd += distance(k_source, k_target, pcl::L2_SQR);
        }
        all_residual = ssd;
      }
      // set our nice results
      context->set("correspondences", correspondences_inlier);
    }
    float score = (float)correspondences_inlier->size() / correspondences->size();

    boost::shared_ptr<typename ContextType::Transformation> trans(new typename ContextType::Transformation(keypoint_transformation.matrix()));

    // calc transformation error
    double rotation_error = -1;
    double translation_error = -1;
    if(this->m_params->get("ransac_calc_transformation_error", true))
    {
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_source =
        context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
      boost::shared_ptr<typename ContextType::Transformation> ground_truth_target =
        context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
      Eigen::Affine3f gt(*ground_truth_source);
      Eigen::Affine3f t((*ground_truth_target) * (*trans));

      // just norm of difference of translation component
      Eigen::Vector3f diff_trans = gt.translation() - t.translation();
      translation_error = diff_trans.norm();

      // angular distance from quaternion representation of rotation component
      Eigen::Quaternionf q1(gt.rotation());
      Eigen::Quaternionf q2(t.rotation());
      rotation_error = q1.angularDistance(q2);
    }
    context->set("keypoint_transformation", trans);

    context->set("ransac_score", score);

#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif


    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, correspondences_inlier->size());
    this->m_logger->set(run, 2, residual);
    this->m_logger->set(run, 3, all_residual);
    this->m_logger->set(run, 4, translation_error);
    this->m_logger->set(run, 5, rotation_error);
    this->m_logger->set(run, 6, score);

    return valid_transform ? correspondences_inlier->size() : 0;
  }

  template<typename P, typename K, typename F>
  std::string Ransac<P, K, F>::getName() const
  {
    return "RA";
  }
}

PCLREF_INSTANTIATE_PRODUCT(Ransac,PCLREF_TYPES_PRODUCT)
