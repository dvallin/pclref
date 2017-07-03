#include <ICP.h>
#include <DataTable.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/filters/voxel_grid.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  ICP<P, K, F>::ICP()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void ICP<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("icp_time");
    names.push_back("icp_score");
    names.push_back("icp_translation_error");
    names.push_back("icp_rotation_error");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  void ICP<P, K, F>::prepareAdditionalLogs()
  {
    this->m_additional_logs.clear();

    bool write = this->m_params->template get<bool>("icp_write", false);

    if(write)
    {
      std::vector<DataTable::columnType> types;
      std::vector<std::string> names;
      types.push_back(DataTable::FLOAT);
      names.push_back("source");
      types.push_back(DataTable::FLOAT);
      names.push_back("target");
      for(int i = 0; i < 16; ++i)
      {
        types.push_back(DataTable::FLOAT);
        names.push_back("t" + precision_cast<int>(i));
      }

      typename DataTable::Ptr log = typename DataTable::Ptr(new DataTable(types, names));
      log->setName("icp_log");
      this->m_additional_logs.push_back(log);
    }
  }

  template<typename P, typename K, typename F>
  int ICP<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
    float ms = 0;
    float score = 0;
    float trans = 0;
    float rot = 0;
    // this step is run accumulated. this means we have a context of contexts containing feature spaces we shall use
    if(context->exists("accu") && context->template get<bool>("accu"))
    {
      typename ContextType::Ptr accu = context->template get<typename ContextType::Ptr>("mstContext", "ceContexts");
      typename ContextType::iterator iterI = accu->begin();
      typename ContextType::iterator iend = accu->end();
      for(; iterI != iend; ++iterI)
      {
        float ms_t = 0;
        float score_t = 0;
        float trans_t = 0;
        float rot_t = 0;
        typename ContextType::Ptr sourceContext = boost::any_cast<typename ContextType::Ptr>(*iterI);
        doICP(sourceContext, ms_t, score_t, trans_t, rot_t);
        ms += ms_t;
        score += score_t;
        trans += trans_t;
        rot += rot_t;
      }
    }
    else
    {
      doICP(context, ms, score, trans, rot);
    }
    this->m_logger->set(run, 0, ms);
    this->m_logger->set(run, 1, score);
    this->m_logger->set(run, 2, trans);
    this->m_logger->set(run, 3, rot);
    return 1;
  }

  template<typename P, typename K, typename F>
  int ICP<P, K, F>::doICP(typename ContextType::Ptr context, float& ms, float& score, float& trans, float& rot)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    bool do_reject = this->m_params->template get<bool>("icp_reject", false);
    double ransac_score_threshold = this->m_params->template get<double>("ransac_score_threshold", 0.1);
    double rscore = 0.99;
    if(context->exists("ransac_score"))
      rscore = context->template get<float>("ransac_score");
    bool reject = ransac_score_threshold < rscore || rscore == 1;

    boost::shared_ptr<typename ContextType::Transformation> keypoint_transformation = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("keypoint_transformation");
    typename ContextType::PointCloud::Ptr output_cloud(new typename ContextType::PointCloud);

    if(!reject || !do_reject)
    {
      typename ContextType::PointCloud::Ptr cloud_source = context->template get<typename ContextType::PointCloud::Ptr>("cloud_source");
      typename ContextType::PointCloud::Ptr cloud_target = context->template get<typename ContextType::PointCloud::Ptr>("cloud_target");

      float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);
      bool removeNans = this->m_params->template get<bool>("icp_remove_nans", true);
      bool downsample = this->m_params->template get<bool>("icp_downsample", false);
      float leaf_size = this->m_params->template get<double>("icp_leaf_size", 0.05);

      bool reciprocal = this->m_params->template get<bool>("icp_reciprocal", true);
      int max_iter = this->m_params->template get<int>("icp_max_iterations", 35);
      double fitness_epsilon = this->m_params->template get<double>("icp_fitness_epsilon", 1e-10);
      double transformation_epsilon = this->m_params->template get<double>("icp_transformation_epsilon", 1e-10);
      double min_overlap_ratio = this->m_params->template get<double>("icp_min_overlap_ratio", 0.1);
      double max_overlap_ratio = this->m_params->template get<double>("icp_max_overlap_ratio", 0.9);
      double distance_threshold = this->m_params->template get<double>("icp_distance_threshold", model_resolution);
      double ransac_inlier_threshold = this->m_params->template get<double>("icp_ransac_inlier_threshold", model_resolution);

      if(removeNans)
      {
        {
          typename ContextType::PointCloud::Ptr cloud_filtered(new typename ContextType::PointCloud);

          for(size_t i = 0; i < cloud_source->points.size(); ++i)
          {
            P p = cloud_source->points[i];
            if(isValid(p))
            {
              cloud_filtered->points.push_back(p);
            }
          }
          cloud_source = cloud_filtered;
        }
        {
          typename ContextType::PointCloud::Ptr cloud_filtered(new typename ContextType::PointCloud);
          for(size_t i = 0; i < cloud_target->points.size(); ++i)
          {
            P p = cloud_target->points[i];
            if(isValid(p))
            {
              cloud_filtered->points.push_back(p);
            }
          }
          cloud_target = cloud_filtered;
        }
      }
      if(downsample)
      {
        pcl::VoxelGrid<P> sor;
        sor.setLeafSize (leaf_size, leaf_size, leaf_size);

        {
          typename ContextType::PointCloud::Ptr filtered_cloud(new typename ContextType::PointCloud);
          sor.setInputCloud (cloud_source);
          sor.filter (*filtered_cloud);
          cloud_source = filtered_cloud;
        }
        {
          typename ContextType::PointCloud::Ptr filtered_cloud(new typename ContextType::PointCloud);
          sor.setInputCloud (cloud_target);
          sor.filter (*filtered_cloud);
          cloud_target = filtered_cloud;
        }

      }
      pcl::IterativeClosestPoint<P, P, float> icp;
      icp.setUseReciprocalCorrespondences(reciprocal);
      icp.setMaximumIterations(max_iter);
      icp.setEuclideanFitnessEpsilon(fitness_epsilon);
      icp.setTransformationEpsilon(transformation_epsilon);
      icp.setMaxCorrespondenceDistance(distance_threshold);
      icp.setRANSACOutlierRejectionThreshold(ransac_inlier_threshold);

      // Add another rejection method
      pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr rej_trimmed(new pcl::registration::CorrespondenceRejectorVarTrimmed());
      rej_trimmed->setMinRatio(min_overlap_ratio);
      rej_trimmed->setMaxRatio(max_overlap_ratio);
      icp.addCorrespondenceRejector(rej_trimmed);

      // Setting point cloud to be aligned.
      icp.setInputSource(cloud_source);
      // Setting point cloud to be aligned to.
      icp.setInputTarget(cloud_target);

      // Calculating required rigid transform to align the input cloud to the target cloud using keypoint_transformation as an initial guess.
      icp.align(*output_cloud, *keypoint_transformation);

      boost::shared_ptr<typename ContextType::Transformation> final_transformation(
        new typename ContextType::Transformation(icp.getFinalTransformation()));

      // calc transformation error
      if(this->m_params->get("icp_calc_transformation_error", true))
      {
        boost::shared_ptr<typename ContextType::Transformation> ground_truth_source =
          context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
        boost::shared_ptr<typename ContextType::Transformation> ground_truth_target =
          context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
        Eigen::Affine3f gt(*ground_truth_source);
        Eigen::Affine3f t((*ground_truth_target) * (*final_transformation));

        // just norm of difference of translation component
        Eigen::Vector3f diff_trans = gt.translation() - t.translation();
        trans = diff_trans.norm();

        // angular distance from quaternion representation of rotation component
        Eigen::Quaternionf q1(gt.rotation());
        Eigen::Quaternionf q2(t.rotation());
        rot = q1.angularDistance(q2);
      }

      score = icp.getFitnessScore();

      bool write = this->m_params->template get<bool>("icp_write", false);

      if(write)
      {
        size_t row = this->m_additional_logs[0]->getRowCount();
        this->m_additional_logs[0]->set(row, 0, context->template get<identifier>("source_id"));
        this->m_additional_logs[0]->set(row, 1, context->template get<identifier>("target_id"));
        this->m_additional_logs[0]->writeEigenMatrix(2, row, *final_transformation, true);
      }
      context->set("output_cloud", output_cloud);
      context->set("final_transformation", final_transformation);
    }
    else
    {
      context->set("output_cloud", output_cloud);
      context->set("final_transformation", keypoint_transformation);
    }
#ifdef PCLREF_PARALLEL
    ms = (double(omp_get_wtime() - t0)*1000);
#else
    ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif
  }

  template<typename P, typename K, typename F>
  std::string ICP<P, K, F>::getName() const
  {
    return "ICP";
  }
}

PCLREF_INSTANTIATE_PRODUCT(ICP,PCLREF_TYPES_PRODUCT)

