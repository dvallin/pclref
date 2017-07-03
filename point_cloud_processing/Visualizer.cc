#include <Visualizer.h>
#include <PclRefLIB.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace pclref
{
  /// see: http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
  /// pretty simple tho
  Eigen::Vector3i getHeatMapColor(float value)
  {
    const int NUM_COLORS = 4;
    static float color[NUM_COLORS][3] = { {0,0,1}, {0,1,0}, {1,1,0}, {1,0,0} };
    // A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.

    int idx1;        // |-- Our desired color will be between these two indexes in "color".
    int idx2;        // |
    float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.

    if(value <= 0)
    {
      idx1 = idx2 = 0;             // accounts for an input <=0
    }
    else if(value >= 1)
    {
      idx1 = idx2 = NUM_COLORS-1;  // accounts for an input >=0
    }
    else
    {
      value = value * (NUM_COLORS-1);        // Will multiply value by 3.
      idx1  = floor(value);                  // Our desired color will be after this index.
      idx2  = idx1+1;                        // ... and before this index (inclusive).
      fractBetween = value - float(idx1);    // Distance between the two indexes (0-1).
    }

    float r = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
    float g = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
    float b = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
    return Eigen::Vector3i(255*r, 255*g, 255*b);
  }

  template<typename P, typename K, typename F>
  Visualizer<P, K, F>::Visualizer()
  {
    init();
    m_full_view = false;
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::init()
  {
    m_viewer.reset(new pcl::visualization::PCLVisualizer);
    m_viewer->setBackgroundColor(0, 0, 0);
    m_correspondences = 0;
  }

  template<typename P, typename K, typename F>
  bool Visualizer<P, K, F>::wasStopped()
  {
    return m_viewer->wasStopped();
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::spinOnce()
  {
    m_viewer->spinOnce(100);
    pcl_sleep(0.01);
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::setFullContext(typename ContextType::Ptr context)
  {
    m_full_context = context;
  }

  template<typename P, typename K, typename F>
  template<typename T>
  void Visualizer<P, K, F>::addCloud(typename pcl::PointCloud<T>::Ptr cloud, const std::string& id, const Eigen::Vector3i& color)
  {
    pcl::visualization::PointCloudColorHandlerCustom<T> color_handler(cloud, color[0], color[1], color[2]);
    Eigen::Vector4f tmpP = cloud->sensor_origin_;
    Eigen::Quaternionf tmpO = cloud->sensor_orientation_;
    cloud->sensor_origin_ = Eigen::Vector4f::Zero();
    cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
    m_viewer->addPointCloud<T>(cloud, color_handler, id);
    cloud->sensor_origin_ = tmpP;
    cloud->sensor_orientation_ = tmpO;
  }

  template<typename P, typename K, typename F>
  template<typename T>
  void Visualizer<P, K, F>::addKeypoints(typename pcl::PointCloud<T>::Ptr cloud, const std::string& id, const Eigen::Vector3i& color)
  {
    addCloud<T>(cloud, id, color);
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
  }

  template<typename P, typename K, typename F>
  template<typename T>
  void Visualizer<P, K, F>::addCorrespondence(const pcl::Correspondence& correspondence,
      typename pcl::PointCloud<T>::Ptr query,
      typename pcl::PointCloud<T>::Ptr match)
  {
    T pn_0 = query->points[correspondence.index_query];
    T pn_1 = match->points[correspondence.index_match];
    Eigen::Vector3f p_0(pn_0.x, pn_0.y, pn_0.z);
    Eigen::Vector3f p_1(pn_1.x, pn_1.y, pn_1.z);

    std::stringstream l_ss;
    l_ss << "correspondence " << m_correspondences++;
    m_viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(p_0[0], p_0[1], p_0[2]),
                                     pcl::PointXYZ(p_1[0], p_1[1], p_1[2]), l_ss.str());
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::visualizerAddCloud(typename ContextType::Ptr context, const std::string& name,
      const typename ContextType::Transformation& transformation,
      const Eigen::Vector3i color)
  {
    if(context->exists(name))
    {
      typename ContextType::PointCloud::Ptr cloud = context->template
          get<typename ContextType::PointCloud::Ptr>(name);
      visualizerAddCloud(cloud, name, transformation, color);
    }
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::visualizerAddCloud(typename ContextType::PointCloud::Ptr cloud, const std::string& name,
      const typename ContextType::Transformation& transformation,
      const Eigen::Vector3i color)
  {
    typename ContextType::PointCloud::Ptr cloud_trans(new typename ContextType::PointCloud);
    pcl::transformPointCloud(*cloud, *cloud_trans, transformation);
    assert(cloud->size() == cloud_trans->size());
    cloud_trans->sensor_orientation_ = cloud->sensor_orientation_;
    cloud_trans->sensor_origin_ = cloud->sensor_origin_;
    addCloud<P>(cloud_trans, name, color);
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::visualizerAddCloudIndexed(
    typename ContextType::Ptr context, const std::string& cloud_name, const std::string& name,
    const typename ContextType::Transformation& transformation,
    const Eigen::Vector3i color)
  {
    if(context->exists(name) && context->exists(cloud_name))
    {
      typename ContextType::PointCloud::Ptr cloud = context->template
          get<typename ContextType::PointCloud::Ptr>(cloud_name);
      typename ContextType::PointCloud::Ptr cloud_trans(new typename ContextType::PointCloud);
      pcl::transformPointCloud(*cloud, *cloud_trans, transformation);
      assert(cloud->size() == cloud_trans->size());
      cloud_trans->sensor_orientation_ = cloud->sensor_orientation_;
      cloud_trans->sensor_origin_ = cloud->sensor_origin_;

      typename ContextType::IndexCloud::Ptr indices = context->template
          get<typename ContextType::IndexCloud::Ptr>(name);
      typename ContextType::PointCloud::Ptr cloud_indexed = typename ContextType::PointCloud::Ptr(
            new typename ContextType::PointCloud);

      for(int i = 0; i < indices->size(); ++i)
        cloud_indexed->points.push_back(cloud_trans->points[indices->points[i]]);

      cloud_indexed->sensor_orientation_ = cloud->sensor_orientation_;
      cloud_indexed->sensor_origin_ = cloud->sensor_origin_;
      addCloud<P>(cloud_indexed, name, color);
    }
  }


  template<typename P, typename K, typename F>
  typename Visualizer<P, K, F>::ContextType::KeypointCloud::Ptr
  Visualizer<P, K, F>::visualizerAddKeypoints(typename ContextType::Ptr context, const std::string& name,
      const typename ContextType::Transformation& transformation,
      const Eigen::Vector3i color)
  {
    typename ContextType::KeypointCloud::Ptr cloud_trans(new typename ContextType::KeypointCloud);
    if(context->exists(name))
    {
      typename ContextType::KeypointCloud::Ptr cloud = context->template
          get<typename ContextType::KeypointCloud::Ptr>(name);
      pcl::transformPointCloud(*cloud, *cloud_trans, transformation);
      assert(cloud->size() == cloud_trans->size());
      addKeypoints<K>(cloud_trans, name, color);
      cloud_trans->sensor_orientation_ = cloud->sensor_orientation_;
      cloud_trans->sensor_origin_ = cloud->sensor_origin_;
    }
    return cloud_trans;
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::visualizeCorrespondences(
    typename ContextType::Ptr context,
    typename ContextType::KeypointCloud::Ptr keypoints_source,
    typename ContextType::KeypointCloud::Ptr keypoints_target)
  {
    if(context->exists("correspondences"))
    {
      boost::shared_ptr<pcl::Correspondences> correspondences = context->template
          get<boost::shared_ptr<pcl::Correspondences> >("correspondences_unfiltered", "correspondences");

      // Visualize correspondence
      BOOST_FOREACH(pcl::Correspondence correspondence, *correspondences)
      {
        addCorrespondence<K>(correspondence, keypoints_source, keypoints_target);
      }
    }
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::createStandardView()
  {
    typename ContextType::iterator iter =(*m_contexts)[m_current_context];
    typename ContextType::Ptr context = boost::any_cast<typename ContextType::Ptr>(*iter);

    visualizerAddCloud(context, "cloud_source", m_source_transformation, Eigen::Vector3i(0, 0, 255));
    visualizerAddCloud(context, "cloud_target", m_target_transformation, Eigen::Vector3i(0, 255, 0));
    //visualizerAddCloudIndexed(context, "cloud_source", "cloud_overlap", m_source_transformation, Eigen::Vector3i(0, 255, 255));

    typename ContextType::KeypointCloud::Ptr k_source, k_target;
    k_source = visualizerAddKeypoints(context, "keypoints_source", m_source_transformation, Eigen::Vector3i(255, 0, 0));
    k_target = visualizerAddKeypoints(context, "keypoints_target", m_target_transformation, Eigen::Vector3i(255, 0, 0));
    visualizeCorrespondences(context, k_source, k_target);

  }
  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::createFullView()
  {
    typename ContextType::iterator iter, iend;
    iend = m_full_context->end();

    std::map<int, typename ContextType::Transformation> poses;
    poses.insert(std::pair<int, typename ContextType::Transformation>
                 (0, ContextType::Transformation::Identity()));
    bool done = false;
    while(!done)
    {
      done = true;
      iter = m_full_context->begin();
      for(; iter != iend; ++iter)
      {
        typename ContextType::Ptr sourceContext = boost::any_cast<typename ContextType::Ptr>(*iter);

        int i = sourceContext->template get<identifier>("source_id") - 1;
        int j = sourceContext->template get<identifier>("target_id") - 1;
        if(poses.find(i) != poses.end()
            && poses.find(j) == poses.end())
        {
          typename ContextType::Transformation trans =
            *sourceContext->template get<boost::shared_ptr<typename ContextType::Transformation> >("final_transformation");
          // trans is source->target
          // source found, so go to pose of source and then apply inverse trans
          poses.insert(std::pair<int, typename ContextType::Transformation>
                       (j, poses[i] * trans.inverse()));
          done = false;
        }
        else if(poses.find(j) != poses.end()
                && poses.find(i) == poses.end())
        {
          typename ContextType::Transformation trans =
            *sourceContext->template get<boost::shared_ptr<typename ContextType::Transformation> >("final_transformation");
          // trans is source->target
          // target pose found, so go with trans to target and then apply pose
          poses.insert(std::pair<int, typename ContextType::Transformation>
                       (i, poses[j] * trans));
          done = false;
        }
      }
    }

    iter = m_full_context->begin();
    int count = poses.size();
    int k = 0;
    std::set<int> added_clouds;
    for(; iter != iend; ++iter)
    {
      typename ContextType::Ptr sourceContext = boost::any_cast<typename ContextType::Ptr>(*iter);
      int i = sourceContext->template get<identifier>("source_id") - 1;
      if(added_clouds.find(i) == added_clouds.end())
      {
        visualizerAddCloud(sourceContext->template get<typename ContextType::PointCloud::Ptr>("cloud_source"),
                           "cloud_" + precision_cast(k, 1),
                           poses[i], getHeatMapColor(((float)k)/count));
        added_clouds.insert(i);
        ++k;
      }

      int j = sourceContext->template get<identifier>("target_id") - 1;
      if(added_clouds.find(j) == added_clouds.end())
      {
        visualizerAddCloud(sourceContext->template get<typename ContextType::PointCloud::Ptr>("cloud_target"),
                           "cloud_" + precision_cast(k, 1),
                           poses[j], getHeatMapColor(((float)k)/count));
        added_clouds.insert(j);
        ++k;
      }
    }
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::startVisualization(typename ContextType::Ptr contexts)
  {
    m_contexts = contexts;
    m_current_context = 0;

    boost::function<void (const pcl::visualization::KeyboardEvent&)> f =
      boost::bind(&Visualizer<P, K, F>::keyboardEventCallback, this, _1);
    m_viewer->registerKeyboardCallback (f);
    m_viewer->setBackgroundColor (1, 1, 1);

    m_source_transformation = Eigen::Matrix4f::Identity();
    m_target_transformation = Eigen::Matrix4f::Identity();
    createStandardView();

    // Loop until user stops the viewer
    while (!wasStopped()) spinOnce();
    m_viewer->close();
  }

  template<typename P, typename K, typename F>
  void Visualizer<P, K, F>::keyboardEventCallback(const pcl::visualization::KeyboardEvent &event)
  {
    typename ContextType::iterator iter =(*m_contexts)[m_current_context];
    typename ContextType::Ptr context = boost::any_cast<typename ContextType::Ptr>(*iter);
    bool update = false;
    if (event.getKeySym () == "v" && event.keyDown ())
    {
      update = true;
      m_source_transformation = *context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_source");
      m_target_transformation = *context->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth_target");
    }
    else if (event.getKeySym () == "b" && event.keyDown ())
    {
      update = true;
      m_source_transformation = *context->template get<boost::shared_ptr<typename ContextType::Transformation> >("final_transformation");
      m_target_transformation = Eigen::Matrix4f::Identity();
    }
    else if (event.getKeySym () == "n" && event.keyDown ())
    {
      update = true;
      m_source_transformation = *context->template get<boost::shared_ptr<typename ContextType::Transformation> >("keypoint_transformation");
      m_target_transformation = Eigen::Matrix4f::Identity();
    }
    else if (event.getKeySym () == "m" && event.keyDown ())
    {
      update = true;
      m_source_transformation = Eigen::Matrix4f::Identity();
      m_target_transformation = Eigen::Matrix4f::Identity();
    }
    else if (event.getKeySym () == "t" && event.keyDown ())
    {
      m_full_view = !m_full_view;
      update = true;
    }
    else if (event.getKeySym () == "minus" && event.keyDown () && m_current_context > 0)
    {
      update = true;
      --m_current_context;
    }
    else if (event.getKeySym () == "plus" && event.keyDown () && m_current_context < m_contexts->size() - 1)
    {
      update = true;
      ++m_current_context;
    }
    if(update)
    {
      m_viewer->removeAllShapes();
      m_viewer->removeAllPointClouds();
      if(m_full_view)
        createFullView();
      else
        createStandardView();
    }
  }
}

PCLREF_INSTANTIATE_PRODUCT(Visualizer,PCLREF_TYPES_PRODUCT)

