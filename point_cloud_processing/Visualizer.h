/*
 * visualizer.h
 *
 *  Created on: May 22, 2014
 *      Author: max
 */

#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include <PclRefLIB.h>
#include <ProcessingStep.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pclref
{
  /**
  * \class Visualizer
  *
  * \brief A visualizer based on the pcl::visualization::PCLVisualizer
  *
  * In pair view the transformations, keypoints and correspondences can be investigated.
  * In full view the registration over all point clouds is rendered.
  *
  * Additional controls:
  *
  * \arg v: show ground truth transformations
  * \arg b: show final transformations
  * \arg n: show keypoint transformations (after RANSAC)
  * \arg m: show identity transformation
  * \arg t: switch to full or pair view
  *
  * run on context with contexts with following objects:
    * \pre ProcessingContext::PointCloud cloud_source
    * \pre ProcessingContext::PointCloud cloud_target
    * \pre ProcessingContext::Correspondences correspondences_unfiltered or correspondences
    * \pre ProcessingContext::KeypointCloud keypoints_source
    * \pre ProcessingContext::KeypointCloud keypoints_target
    * \pre ProcessingContext::Transformation keypoint_transformation
    * \post ProcessingContext::Transformation final_transformation
    *
    * the full context must be an mstContext (a subset of ceContexts)
    * \see pclref::MST
    * \see pclref::Ransac
    * \see pclref::ICP
  *
  */
  template<typename PointType, typename KeypointType, typename FeatureType>
  class Visualizer
  {
  public:
    typedef ProcessingContext<PointType, KeypointType, FeatureType> ContextType; ///< The Context

    /// standard constructor
    Visualizer();

    /// initialize Visualizer, already called by constructor
    void init();

    /// true if visualizer was stopped
    bool wasStopped();

    /// do one rendering loop
    void spinOnce();

    /// set the mstContext for a full view of the scene
    void setFullContext(typename ContextType::Ptr context);

    /// build a pair-wise view
    void createStandardView();

    /// build a full view of the scene
    void createFullView();

    /// start the visualization, this is blocking
    void startVisualization(typename ContextType::Ptr contexts);

    /// add cloud from this context
    void visualizerAddCloud(typename ContextType::Ptr context, const std::string& name,
                            const typename ContextType::Transformation& transformation,
                            const Eigen::Vector3i color);

    /// add cloud
    void visualizerAddCloud(typename ContextType::PointCloud::Ptr cloud, const std::string& name,
                            const typename ContextType::Transformation& transformation,
                            const Eigen::Vector3i color);

    /// add cloud from this context
    void visualizerAddCloudIndexed(typename ContextType::Ptr context, const std::string& cloud_name, const std::string& name,
                                   const typename ContextType::Transformation& transformation,
                                   const Eigen::Vector3i color);

    /// add keypoints from this context
    typename ContextType::KeypointCloud::Ptr
    visualizerAddKeypoints(typename ContextType::Ptr context, const std::string& name,
                           const typename ContextType::Transformation& transformation,
                           const Eigen::Vector3i color);

    /// add all correspondences from this context
    void visualizeCorrespondences(typename ContextType::Ptr context,
                                  typename ContextType::KeypointCloud::Ptr keypoints_source,
                                  typename ContextType::KeypointCloud::Ptr keypoints_target);


    /// add a point cloud
    template<typename T>
    void addCloud(typename pcl::PointCloud<T>::Ptr cloud, const std::string& id, const Eigen::Vector3i& color);

    /// add a keypoint cloud
    template<typename T>
    void addKeypoints(typename pcl::PointCloud<T>::Ptr cloud, const std::string& id, const Eigen::Vector3i& color);

    /// add a correspondence
    template<typename T>
    void addCorrespondence(const pcl::Correspondence& correspondence,
                           typename pcl::PointCloud<T>::Ptr query,
                           typename pcl::PointCloud<T>::Ptr match);

    /// handle keyboard events
    void keyboardEventCallback(const pcl::visualization::KeyboardEvent &event);

  private:
    typename ContextType::Transformation m_source_transformation;
    typename ContextType::Transformation m_target_transformation;
    unsigned int m_correspondences;
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    unsigned int m_current_context;
    typename ContextType::Ptr m_contexts;

    bool m_full_view;
    typename ContextType::Ptr m_full_context;
  };
}

#define PCLREF_INSTANTIATE_Visualizer(T,K,F) template class pclref::Visualizer <T,K,F>;

#endif

