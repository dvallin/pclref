#include <TransformCloud.h>
#include <DataTable.h>
#include <MersenneTwister.h>

#include <pcl/io/pcd_io.h>

namespace pclref
{
  template<typename P, typename K, typename F>
  TransformCloud<P, K, F>::TransformCloud()
  {
    init();
  }

  template<typename P, typename K, typename F>
  void TransformCloud<P, K, F>::init()
  {
    BaseType::init();

    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    std::vector<std::string> names;
    names.push_back("tc_time");
    this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
  }

  template<typename P, typename K, typename F>
  int TransformCloud<P, K, F>::compute(size_t run, typename ContextType::Ptr context)
  {
#ifdef PCLREF_PARALLEL
    double t0 = omp_get_wtime();
#else
    const clock_t t0 = clock();
#endif

    typename ContextType::PointCloud::Ptr cloud = context->template get<typename ContextType::PointCloud::Ptr> ("cloud");
    bool apply = this->m_params->template get<bool>("tc_apply", true);
    bool demean = this->m_params->template get<bool>("tc_demean", true);

    if(apply)
    {
      Eigen::Transform<float, 3, Eigen::Affine> transform;
      Eigen::AngleAxisf rotation;
      if(demean)
      {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*cloud, centroid);
        transform = Eigen::Translation3f((-centroid).head<3>());
        rotation = Eigen::AngleAxisf::Identity();
      }
      else
      {
        double dX = this->m_params->template get<double>("tc_dx", 5.0);
        double dY = this->m_params->template get<double>("tc_dy", 5.0);
        double dZ = this->m_params->template get<double>("tc_dz", 5.0);
        bool pitch_lock = this->m_params->template get<bool>("pitch_lock", true);
        bool yaw_lock = this->m_params->template get<bool>("yaw_lock", false);
        bool roll_lock = this->m_params->template get<bool>("roll_lock", true);

        Eigen::Vector4f offset;
        offset[0] = MersenneTwister::next(-dX, dX);
        offset[1] = MersenneTwister::next(-dY, dY);
        offset[2] = MersenneTwister::next(-dZ, dZ);
        offset[3] = 0;

        double pitch = pitch_lock ? 0 : MersenneTwister::next(-1, 1);
        double yaw = yaw_lock ? 0 : MersenneTwister::next(-1, 1);
        double roll = roll_lock ? 0 : MersenneTwister::next(-1, 1);

        Eigen::AngleAxisf pitchAngle(pitch*M_PI, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf yawAngle(yaw*M_PI, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rollAngle(roll*M_PI, Eigen::Vector3f::UnitZ());
        rotation = yawAngle * pitchAngle * rollAngle;

        transform = Eigen::Translation3f(offset.head<3>()) * rotation;
      }

      typename ContextType::PointCloud::Ptr cloud_t(new typename ContextType::PointCloud);
      pcl::transformPointCloud(*cloud, *cloud_t, transform);
      assert(cloud->size() == cloud_t->size());

      cloud_t->sensor_orientation_ = rotation*cloud->sensor_orientation_;
      cloud_t->sensor_origin_ = transform*cloud->sensor_origin_;

      if(context->exists("ground_truth"))
      {
        boost::shared_ptr<typename ContextType::Transformation> ground_truth = context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth");

        *ground_truth = (*ground_truth) * (transform.matrix().inverse());
        context->set("ground_truth", ground_truth);
      }

      context->set("cloud", cloud_t);
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
  std::string TransformCloud<P, K, F>::getName() const
  {
    return "TC";
  }
}

PCLREF_INSTANTIATE_PRODUCT(TransformCloud,PCLREF_TYPES_PRODUCT)
