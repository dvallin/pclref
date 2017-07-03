/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_IMPROVED_H_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_IMPROVED_H_

#include <string>

#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/search/flann_search.h>
#include <pcl/pcl_macros.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_types.h>

#ifdef USE_NEW_PCL_FEATURES
#include <pcl/search/impl/flann_search.h>
#define FLANN_SEARCH_CLASS pcl::search::FlannSearch
#else
#include <flann_search_improved.h>
#define FLANN_SEARCH_CLASS pcl::search::FlannSearchImproved
#endif

namespace pcl
{
  namespace registration
  {
    /** \brief @b CorrespondenceEstimationImproved represents the base
      * class for determining correspondences between target and query
      * point sets/features.  This improves over the default version
      * by looking at more than just the nearest neighbor.
      *
      * Code example:
      *
      * \code
      * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source, target;
      * // ... read or fill in source and target
      * pcl::CorrespondenceEstimationImproved<pcl::PointXYZ, pcl::PointXYZ> est;
      * est.setInputSource (source);
      * est.setInputTarget (target);
      *
      * pcl::Correspondences all_correspondences;
      * // Determine all reciprocal correspondences
      * est.determineReciprocalCorrespondences (all_correspondences);
      * \endcode
      *
      * \author Radu B. Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float, typename Metric = flann::L2_Simple<float> >
    class CorrespondenceEstimationImproved : public CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>
    {
    public:
      typedef boost::shared_ptr<CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar> > Ptr;
      typedef boost::shared_ptr<const CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar> > ConstPtr;

      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;
      using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::force_no_recompute_;
      using PCLBase<PointSource>::deinitCompute;

      typedef FLANN_SEARCH_CLASS<PointTarget, Metric> FlannTree;
      typedef typename FLANN_SEARCH_CLASS<PointTarget, Metric>::Ptr FlannTreePtr;
      typedef FLANN_SEARCH_CLASS<PointSource, Metric> FlannTreeReciprocal;
      typedef typename FLANN_SEARCH_CLASS<PointSource, Metric>::Ptr FlannTreeReciprocalPtr;

      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

      enum IndexCreator
      {
        KDTree = 0,
        MultiKDTree = 1,
        KMeans = 2,
        Linear = 3,
        Composite = 4,
        Lsh = 5,
        Auto = 6
      };

      /** \brief Empty constructor. */
      CorrespondenceEstimationImproved ()
        : flann_tree_updated_(true),
          flann_tree_reciprocal_updated_(true)
      {
        corr_name_  = "CorrespondenceEstimationImproved";
        setFlannParameters();
        setKDForestParameters();
        setKMeansParameters();
        setLshParameters();
        setAutoTuneParameters();
      }

      /** \brief Empty destructor */
      virtual ~CorrespondenceEstimationImproved () {}


      /** \brief Determine the correspondences between input and target cloud.
        * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
        * \param[in] max_distance maximum allowed distance between correspondences
        */
      virtual void
      determineCorrespondences (pcl::Correspondences &correspondences,
                                double max_distance = std::numeric_limits<double>::max ())
      {
        determineCorrespondences(correspondences, max_distance, 1);
      }

      /** \brief Determine the reciprocal correspondences between input and target cloud.
        * A correspondence is considered reciprocal if both Src_i has Tgt_i as a
        * correspondence, and Tgt_i has Src_i as one.
        *
        * \param[out] correspondences the found correspondences (index of query and target point, distance)
        * \param[in] max_distance maximum allowed distance between correspondences
        */
      virtual void
      determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                          double max_distance = std::numeric_limits<double>::max ())
      {
        determineReciprocalCorrespondences(correspondences, max_distance, 1);
      }

      virtual void
      determineCorrespondences (pcl::Correspondences &correspondences,
                                double max_distance, int num_neighbours);

      virtual void
      determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                          double max_distance, int num_neighbours);

      virtual boost::shared_ptr< CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > clone () const
      {
        CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar, Metric>*
        e = new CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar, Metric>();
        e->setInputSource(input_);
        e->setInputTarget(target_);
        return boost::shared_ptr< CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > (e);
      }

      void setFlannParameters(IndexCreator idx_ctor = MultiKDTree, int checks = 256, float epsilon = 0)
      {
        idx_ctor_ = idx_ctor;
        checks_ = checks;
        epsilon_ = epsilon;
      }

      void setKDForestParameters(int trees = 4)
      {
        trees_ = trees;
      }

      void setKMeansParameters(int branching = 32, int iterations = 11,
                               flann::flann_centers_init_t centers_init = flann::FLANN_CENTERS_RANDOM, float cb_index = 0.2f)
      {
        branching_ = branching;
        iterations_ = iterations;
        centers_init_ = centers_init;
        cb_index_ = cb_index;
      }

      void setLshParameters(unsigned int table_number = 20u, unsigned int key_size = 16u,
                            unsigned int multi_probe_level = 2u)
      {
        table_number_ = table_number;
        key_size_ = key_size;
        multi_probe_level_ = multi_probe_level;
      }

      void setAutoTuneParameters(float target_precision = 0.9, float build_weight = 0.01,
                                 float memory_weight = 0, float sample_fraction = 0.1)
      {
        target_precision_ = target_precision;
        build_weight_ = build_weight;
        memory_weight_ = memory_weight;
        sample_fraction_ = sample_fraction;
      }

    private:
      bool initFlannCompute ();
      bool initFlannComputeReciprocal ();
      FlannTreePtr createFlannTree();

      bool flann_tree_updated_;
      bool flann_tree_reciprocal_updated_;
      FlannTreePtr flann_tree_;
      FlannTreePtr flann_tree_reciprocal_;

      // FLANN PARAMETERS
      int checks_;
      float epsilon_;
      IndexCreator idx_ctor_;
      // kdforests
      int trees_;
      // kmeans
      int branching_;
      int iterations_;
      flann::flann_centers_init_t centers_init_;
      float cb_index_;
      // lsh
      unsigned int table_number_;
      unsigned int key_size_;
      unsigned int multi_probe_level_;
      // autotuned
      float target_precision_;
      float build_weight_;
      float memory_weight_;
      float sample_fraction_;
    };
  }
}

#include <correspondence_estimation_improved.hpp>

#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_IMPROVED_H_ */
