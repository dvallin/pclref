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
#ifndef PCL_GENERIC_RANSAC_CORRESPONDENCE_REJECTOR_H_
#define PCL_GENERIC_RANSAC_CORRESPONDENCE_REJECTOR_H_

#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/rmsac.h>

namespace pcl
{
  namespace registration
  {
    template <typename PointT>
    class GenericRansacCorrespondenceRejector : public CorrespondenceRejectorSampleConsensus<PointT>
    {
    public:
      using CorrespondenceRejectorSampleConsensus<PointT>::input_;
      using CorrespondenceRejectorSampleConsensus<PointT>::target_;
      using CorrespondenceRejectorSampleConsensus<PointT>::getClassName;
      using CorrespondenceRejectorSampleConsensus<PointT>::inlier_threshold_;
      using CorrespondenceRejectorSampleConsensus<PointT>::max_iterations_;
      using CorrespondenceRejectorSampleConsensus<PointT>::best_transformation_;
      using CorrespondenceRejectorSampleConsensus<PointT>::refine_;
      using CorrespondenceRejectorSampleConsensus<PointT>::save_inliers_;
      using CorrespondenceRejectorSampleConsensus<PointT>::inlier_indices_;

      int em_iterations_;
      double sample_percentage_;
      double probability_;

      GenericRansacCorrespondenceRejector()
        : em_iterations_(3), sample_percentage_(10.0)
      {

      }

      template<typename RansacMethod, typename RansacModel>
      void getRemainingCorrespondences (const pcl::Correspondences& original_correspondences,
                                        pcl::Correspondences& remaining_correspondences,
                                        bool onCorrespondenceHint)
      {
        if (!input_)
        {
          PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No input cloud dataset was given!\n", getClassName ().c_str ());
          return;
        }

        if (!target_)
        {
          PCL_ERROR ("[pcl::registration::%s::getRemainingCorrespondences] No input target dataset was given!\n", getClassName ().c_str ());
          return;
        }

        int nr_correspondences = static_cast<int> (original_correspondences.size ());
        std::vector<int> source_indices(nr_correspondences);
        std::vector<int> target_indices(nr_correspondences);

        // Copy the query-match indices
        for (size_t i = 0; i < original_correspondences.size (); ++i)
        {
          source_indices[i] = original_correspondences[i].index_query;
          target_indices[i] = original_correspondences[i].index_match;
        }

        {
          // From the set of correspondences found, attempt to remove outliers
          // Create the registration model
          typedef typename RansacModel::Ptr SampleConsensusModelRegistrationPtr;
          SampleConsensusModelRegistrationPtr model;
          model.reset (new RansacModel (input_, source_indices));
          // Pass the target_indices
          model->setInputTarget (target_, target_indices);
          // Create a RANSAC model
          RansacMethod sac (model, inlier_threshold_);
          sac.setMaxIterations (max_iterations_);
          sac.setProbability (probability_);
          setParams(sac);

          // Compute the set of inliers
          if (!sac.computeModel ())
          {
            remaining_correspondences = original_correspondences;
            best_transformation_.setIdentity ();
            return;
          }
          else
          {
            if (refine_ && !sac.refineModel ())
            {
              PCL_ERROR ("[pcl::registration::CorrespondenceRejectorSampleConsensus::getRemainingCorrespondences] Could not refine the model! Returning an empty solution.\n");
              return;
            }

            std::vector<int> inliers;
            sac.getInliers (inliers);

            if (inliers.size () < 3)
            {
              remaining_correspondences = original_correspondences;
              best_transformation_.setIdentity ();
              return;
            }

            if(onCorrespondenceHint)
            {
              remaining_correspondences.resize (inliers.size ());
              for(size_t i = 0; i < inliers.size(); ++i)
                remaining_correspondences[i] = original_correspondences[inliers[i]];

              if (save_inliers_)
              {
                inlier_indices_.reserve (inliers.size ());
                for (size_t i = 0; i < inliers.size (); ++i)
                  inlier_indices_.push_back (original_correspondences[inliers[i]].index_query);
              }
            }
            else
            {
              boost::unordered_map<int, int> index_to_correspondence;
              for (int i = 0; i < nr_correspondences; ++i)
                index_to_correspondence[original_correspondences[i].index_query] = i;

              remaining_correspondences.resize (inliers.size ());
              for (size_t i = 0; i < inliers.size (); ++i)
                remaining_correspondences[i] = original_correspondences[index_to_correspondence[inliers[i]]];

              if (save_inliers_)
              {
                inlier_indices_.reserve (inliers.size ());
                for (size_t i = 0; i < inliers.size (); ++i)
                  inlier_indices_.push_back (index_to_correspondence[inliers[i]]);
              }
            }

            // get best transformation
            Eigen::VectorXf model_coefficients;
            sac.getModelCoefficients (model_coefficients);
            best_transformation_.row (0) = model_coefficients.segment<4>(0);
            best_transformation_.row (1) = model_coefficients.segment<4>(4);
            best_transformation_.row (2) = model_coefficients.segment<4>(8);
            best_transformation_.row (3) = model_coefficients.segment<4>(12);
          }
        }
      }

      void setEmIterations(int iterations)
      {
        em_iterations_ = iterations;
      }
      void setSamplePercentage(double sample_percentage)
      {
        sample_percentage_ = sample_percentage;
      }
      void setProbability(double probability)
      {
        probability_ = probability;
      }

      void setParams(pcl::RandomizedRandomSampleConsensus<PointT>& method)
      {
        method.setFractionNrPretest(sample_percentage_);
      }
      void setParams(pcl::RandomizedMEstimatorSampleConsensus<PointT>& method)
      {
        method.setFractionNrPretest(sample_percentage_);
      }
      void setParams(pcl::MaximumLikelihoodSampleConsensus<PointT>& method)
      {
        method.setEMIterations(em_iterations_);
      }
      template<typename AnyOther>
      void setParams(AnyOther& /*method*/)
      {
      }
    };
  }
}

#endif    // PCL_GENERIC_RANSAC_CORRESPONDENCE_REJECTOR_H_

