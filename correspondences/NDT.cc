#include <NDT.h>
#include <DataTable.h>

#include <pcl/registration/ndt.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  NDT<P, K, F>::NDT()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void NDT<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back ("ndt_time");
    names.push_back ("ndt_prob");
    names.push_back ("ndt_translation_error");
    names.push_back ("ndt_rotation_error");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int NDT<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif
    typename ContextType::PointCloud::Ptr cloud_source = context->template get<typename ContextType::PointCloud::Ptr>("cloud_source");
    boost::shared_ptr<typename ContextType::Transformation> keypoint_transformation = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("keypoint_transformation");

    if(!context->exists("reject") || !context->template get<bool>("reject"))
    {
      typename ContextType::PointCloud::Ptr cloud_target = context->template get<typename ContextType::PointCloud::Ptr>("cloud_target");
      typename ContextType::PointCloud::Ptr output_cloud(new typename ContextType::PointCloud);

      float model_resolution = this->m_params->template get<double>("model_resolution", 0.1);
      bool removeNans = this->m_params->template get<bool>("ndt_remove_nans", true);
      bool downsample = this->m_params->template get<bool>("ndt_downsample", false);
      float leaf_size = this->m_params->template get<double>("ndt_leaf_size", 0.05);

      int max_iter = this->m_params->template get<int>("ndt_max_iterations", 35);
      double resolution = this->m_params->template get<double>("ndt_resolution", 10*model_resolution);
      double transformation_epsilon = this->m_params->template get<double>("ndt_transformation_epsilon", 0.01);
      double step_size = this->m_params->template get<double>("ndt_step_size", 0.1);

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
          cloud_target = cloud_filtered;
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
          cloud_source = cloud_filtered;
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
      // Initializing Normal Distributions Transform (NDT).
      pcl::NormalDistributionsTransform<P, P> ndt;

      // Setting scale dependent NDT parameters
      // Setting minimum transformation difference for termination condition.
      ndt.setTransformationEpsilon (transformation_epsilon);
      // Setting maximum step size for More-Thuente line search.
      ndt.setStepSize (step_size);
      //Setting Resolution of NDT grid structure (VoxelGridCovariance).
      ndt.setResolution (resolution);

      // Setting max number of registration iterations.
      ndt.setMaximumIterations (max_iter);

      // Setting point cloud to be aligned.
      ndt.setInputSource (cloud_source);
      // Setting point cloud to be aligned to.
      ndt.setInputTarget (cloud_target);

      // Calculating required rigid transform to align the input cloud to the target cloud.
      ndt.align (*output_cloud, *keypoint_transformation);

      boost::shared_ptr<typename ContextType::Transformation> final_transformation(
        new typename ContextType::Transformation(ndt.getFinalTransformation()));

      // calc transformation error
      double rotation_error = -1;
      double translation_error = -1;
      if(this->m_params->get("ndt_calc_transformation_error", true))
      {
        boost::shared_ptr<typename ContextType::Transformation> ground_truth_source =
          context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
        boost::shared_ptr<typename ContextType::Transformation> ground_truth_target =
          context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
        Eigen::Affine3f gt(*ground_truth_source);
        Eigen::Affine3f t((*ground_truth_target) * (*final_transformation));

        // just norm of difference of translation component
        Eigen::Vector3f diff_trans = gt.translation() - t.translation();
        translation_error = diff_trans.norm();

        // angular distance from quaternion representation of rotation component
        Eigen::Quaternionf q1(gt.rotation());
        Eigen::Quaternionf q2(t.rotation());
        rotation_error = q1.angularDistance(q2);
      }

      context->set("output_cloud", output_cloud);
      context->set("final_transformation", final_transformation);
      this->m_logger->set(run, 1, ndt.getTransformationProbability ());
      this->m_logger->set(run, 2, translation_error);
      this->m_logger->set(run, 3, rotation_error);
    }
    else
    {
      context->set("output_cloud", cloud_source);
      context->set("final_transformation", keypoint_transformation);
      this->m_logger->set(run, 1, -1.0);
      this->m_logger->set(run, 2, -1.0);
      this->m_logger->set(run, 3, -1.0);
    }
#ifdef PCLREF_PARALLEL
    double ms = (double(omp_get_wtime() - t0)*1000);
#else
    double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
#endif

    this->m_logger->set(run, 0, ms);

    return 1;
  }

  template<typename P, typename K, typename F>
  std::string NDT<P, K, F>::getName() const
  {
    return "NDT";
  }
}

PCLREF_INSTANTIATE_PRODUCT(NDT,PCLREF_TYPES_PRODUCT)
