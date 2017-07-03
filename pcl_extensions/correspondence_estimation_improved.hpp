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
#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_IMPROVED_HPP_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_IMPROVED_HPP_

#include <correspondence_estimation_improved.h>
#include <pcl/common/concatenate.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/impl/pcl_base.hpp>

// a bit inconvient here
// see: http://www.pcl-users.org/FLANN-undefined-reference-error-with-pcl-on-android-td4034433.html
// and: http://www.pcl-users.org/KdTreeFLANN-td4019706.html
//#define USE_NEW_PCL_FEATURES

#ifdef USE_NEW_PCL_FEATURES
#include <pcl/search/impl/flann_search.hpp>
#else
#include <flann_search_improved.hpp>
#endif

namespace pcl
{
  namespace registration
  {

    template <typename PointSource, typename PointTarget, typename Scalar, typename Metric>
    typename CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar, Metric>::FlannTreePtr
    CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar, Metric>::createFlannTree()
    {
      FlannTreePtr result;
      typedef typename FlannTree::FlannIndexCreatorPtr CreatorPtrT;
      typedef typename FlannTree::PointRepresentationPtr RepresentationPtrT;

      switch(idx_ctor_)
      {
      case KDTree:
        result = FlannTreePtr(new FlannTree(true, CreatorPtrT(new typename FlannTree::KdTreeIndexCreator())));
        break;
      case MultiKDTree:
        result = FlannTreePtr(new FlannTree(true, CreatorPtrT(new typename FlannTree::KdTreeMultiIndexCreator(4))));
        break;
#ifndef USE_NEW_PCL_FEATURES
      case KMeans:
        result = FlannTreePtr(new FlannTree(true, CreatorPtrT(new typename FlannTree::KMeansIndexCreator(branching_, iterations_, centers_init_, cb_index_))));
        break;
      case Linear:
        result = FlannTreePtr(new FlannTree(true, CreatorPtrT(new typename FlannTree::LinearIndexCreator())));
        break;
      case Composite:
        result = FlannTreePtr(new FlannTree(true, CreatorPtrT(new typename FlannTree::CompositeIndexCreator(trees_, branching_, iterations_, centers_init_, cb_index_))));
        break;
      case Lsh:
        result = FlannTreePtr(new FlannTree(true, CreatorPtrT(new typename FlannTree::LshIndexCreator(table_number_, key_size_, multi_probe_level_))));
        break;
      case Auto:
        result = FlannTreePtr(new FlannTree(true, CreatorPtrT(new typename FlannTree::AutotunedIndexCreator(target_precision_, build_weight_, memory_weight_, sample_fraction_))));
        break;
#endif
      default:
        result = FlannTreePtr(new FlannTree(true, CreatorPtrT(new typename FlannTree::KdTreeMultiIndexCreator(4))));
        break;
      }
      result->setPointRepresentation (RepresentationPtrT (new pcl::DefaultFeatureRepresentation<PointSource>));
      result->setChecks (checks_);
      result->setEpsilon (epsilon_);
      return result;
    }

///////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointSource, typename PointTarget, typename Scalar, typename Metric> bool
    CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar, Metric>::initFlannCompute ()
    {
      if (!target_)
      {
        PCL_ERROR ("[%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
        return (false);
      }

      // Only update target kd-tree if a new target cloud was set
      if (flann_tree_updated_ && !force_no_recompute_)
      {
        flann_tree_ = createFlannTree();

        // If the target indices have been given via setIndicesTarget
        if (target_indices_)
          flann_tree_->setInputCloud (target_, target_indices_);
        else
          flann_tree_->setInputCloud (target_);

        flann_tree_updated_ = false;
      }
      return true;
    }
    template <typename PointSource, typename PointTarget, typename Scalar, typename Metric> bool
    CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar, Metric>::initFlannComputeReciprocal ()
    {
      if (!target_)
      {
        PCL_ERROR ("[%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
        return (false);
      }

      // Only update target kd-tree if a new target cloud was set
      if (flann_tree_reciprocal_updated_ && !force_no_recompute_)
      {
        flann_tree_reciprocal_ = createFlannTree();

        // If the target indices have been given via setIndicesTarget
        if (indices_)
          flann_tree_reciprocal_->setInputCloud (input_, indices_);
        else
          flann_tree_reciprocal_->setInputCloud (input_);

        flann_tree_reciprocal_updated_ = false;
      }

      return true;
    }
///////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointSource, typename PointTarget, typename Scalar, typename Metric> void
    CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar, Metric>::determineCorrespondences (
      pcl::Correspondences &correspondences, double max_distance, int num_neighbours)
    {
      if (!initCompute ())
        return;


      initFlannCompute ();

      typedef typename pcl::traits::fieldList<PointTarget>::type FieldListTarget;
      correspondences.clear();

      std::vector<int> index (num_neighbours);
      std::vector<float> distance (num_neighbours);
      pcl::Correspondence corr;

      // Check if the template types are the same. If true, avoid a copy.
      // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
      if (isSamePointType<PointSource, PointTarget> ())
      {
        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
        {
          flann_tree_->nearestKSearch (input_->points[*idx], num_neighbours, index, distance);
          for (unsigned j=0; j<num_neighbours; ++j)
          {
            if (distance[j] > max_distance)
              continue;

            corr.index_query = *idx;
            corr.index_match = index[j];
            corr.distance = distance[j];
            correspondences.push_back(corr);
          }
        }
      }
      else
      {
        PointTarget pt;

        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
        {
          // Copy the source data to a target PointTarget format so we can search in the tree
          pcl::for_each_type <FieldListTarget> (pcl::NdConcatenateFunctor <PointSource, PointTarget> (
                                                  input_->points[*idx],
                                                  pt));

          flann_tree_->nearestKSearch (pt, num_neighbours, index, distance);
          for (unsigned j=0; j<num_neighbours; ++j)
          {
            if (distance[j] > max_distance)
              continue;

            corr.index_query = *idx;
            corr.index_match = index[j];
            corr.distance = distance[j];
            correspondences.push_back(corr);
          }
        }
      }
      deinitCompute ();
    }

///////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointSource, typename PointTarget, typename Scalar, typename Metric> void
    CorrespondenceEstimationImproved<PointSource, PointTarget, Scalar, Metric>::determineReciprocalCorrespondences (
      pcl::Correspondences &correspondences, double max_distance, int num_neighbours)
    {
      if (!initCompute ())
        return;
      // setup tree for reciprocal search
      // Set the internal point representation of choice
      if (!initComputeReciprocal())
        return;
      initFlannCompute ();
      initFlannComputeReciprocal ();

      typedef typename pcl::traits::fieldList<PointSource>::type FieldListSource;
      typedef typename pcl::traits::fieldList<PointTarget>::type FieldListTarget;
      typedef typename pcl::intersect<FieldListSource, FieldListTarget>::type FieldList;


      correspondences.clear();
      std::vector<int> index (num_neighbours);
      std::vector<float> distance (num_neighbours);
      std::vector<int> index_reciprocal (num_neighbours);
      std::vector<float> distance_reciprocal (num_neighbours);
      pcl::Correspondence corr;
      int target_idx = 0;

      // Check if the template types are the same. If true, avoid a copy.
      // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
      if (isSamePointType<PointSource, PointTarget> ())
      {
        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
        {
          flann_tree_->nearestKSearch (input_->points[*idx], num_neighbours, index, distance);
          for (unsigned j=0; j<num_neighbours; ++j)
          {
            if (distance[j] > max_distance)
              continue;

            target_idx = index[j];

            flann_tree_reciprocal_->nearestKSearch (target_->points[target_idx], num_neighbours, index_reciprocal, distance_reciprocal);
            for (unsigned k=0; k<num_neighbours; ++k)
            {
              if (*idx != index_reciprocal[k])
                continue;

              corr.index_query = *idx;
              corr.index_match = index[j];
              corr.distance = distance[j];
              correspondences.push_back(corr);
            }
          }
        }
      }
      else
      {
        PointTarget pt_src;
        PointSource pt_tgt;

        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
        {
          // Copy the source data to a target PointTarget format so we can search in the tree
          pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointSource, PointTarget> (
                                            input_->points[*idx],
                                            pt_src));

          flann_tree_->nearestKSearch (pt_src, num_neighbours, index, distance);
          for (unsigned j=0; j<num_neighbours; ++j)
          {
            if (distance[j] > max_distance)
              continue;

            target_idx = index[j];

            // Copy the target data to a target PointSource format so we can search in the tree_reciprocal
            pcl::for_each_type<FieldList> (pcl::NdConcatenateFunctor <PointTarget, PointSource> (
                                             target_->points[target_idx],
                                             pt_tgt));

            flann_tree_reciprocal_->nearestKSearch (pt_tgt, num_neighbours, index_reciprocal, distance_reciprocal);
            for (unsigned k=0; k<num_neighbours; ++k)
            {
              if (*idx != index_reciprocal[k])
                continue;

              corr.index_query = *idx;
              corr.index_match = index[j];
              corr.distance = distance[j];
              correspondences.push_back(corr);
            }
          }
        }
      }
      deinitCompute ();
    }

//#define PCL_INSTANTIATE_CorrespondenceEstimationImproved(T,U) template class PCL_EXPORTS CorrespondenceEstimation<T,U>;
  }
}

#endif /* PCL_REGISTRATION_IMPL_CORRESPONDENCE_ESTIMATION_H_ */
