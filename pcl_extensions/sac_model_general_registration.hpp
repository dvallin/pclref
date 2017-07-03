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

#ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_GENERAL_REGISTRATION_H_
#define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_GENERAL_REGISTRATION_H_

#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelGeneralRegistration<PointT>::isSampleGood (const std::vector<int> &samples) const
{
  using namespace pcl::common;
  using namespace pcl::traits;

  int s0 = (*indices_src_)[samples[0]];
  int s1 = (*indices_src_)[samples[1]];
  int s2 = (*indices_src_)[samples[2]];
  int t0 = (*indices_tgt_)[samples[0]];
  int t1 = (*indices_tgt_)[samples[1]];
  int t2 = (*indices_tgt_)[samples[2]];

  PointT p10 = input_->points[s1] - input_->points[s0];
  PointT p20 = input_->points[s2] - input_->points[s0];
  PointT p21 = input_->points[s2] - input_->points[s1];
  PointT q10 = target_->points[t1] - target_->points[t0];
  PointT q20 = target_->points[t2] - target_->points[t0];
  PointT q21 = target_->points[t2] - target_->points[t1];

  return ((p10.x * p10.x + p10.y * p10.y + p10.z * p10.z) > sample_dist_thresh_ &&
          (p20.x * p20.x + p20.y * p20.y + p20.z * p20.z) > sample_dist_thresh_ &&
          (p21.x * p21.x + p21.y * p21.y + p21.z * p21.z) > sample_dist_thresh_ &&
          (q10.x * q10.x + q10.y * q10.y + q10.z * q10.z) > sample_dist_thresh_ &&
          (q20.x * q20.x + q20.y * q20.y + q20.z * q20.z) > sample_dist_thresh_ &&
          (q21.x * q21.x + q21.y * q21.y + q21.z * q21.z) > sample_dist_thresh_);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelGeneralRegistration<PointT>::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelGeneralRegistration::computeModelCoefficients] No target dataset given!\n");
    return (false);
  }
  // Need 3 samples
  if (samples.size () != 3)
    return (false);


  estimateRigidTransformationSVD (*input_, *target_, samples, model_coefficients);
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelGeneralRegistration<PointT>::getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)
{
  if (indices_src_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelGeneralRegistration::getDistancesToModel] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    distances.clear ();
    return;
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelGeneralRegistration::getDistanceToModel] No target dataset given!\n");
    return;
  }
  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    distances.clear ();
    return;
  }
  distances.resize (indices_src_->size ());

  // Get the 4x4 transformation
  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    int p0 = (*indices_src_)[i];
    int p1 = (*indices_tgt_)[i];
    Eigen::Vector4f pt_src (input_->points[p0].x,
                            input_->points[p0].y,
                            input_->points[p0].z, 1);
    Eigen::Vector4f pt_tgt (target_->points[p1].x,
                            target_->points[p1].y,
                            target_->points[p1].z, 1);

    Eigen::Vector4f p_tr (transform * pt_src);
    // Calculate the distance from the transformed point to its correspondence
    // need to compute the real norm here to keep MSAC and friends general
    distances[i] = (p_tr - pt_tgt).norm ();
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelGeneralRegistration<PointT>::selectWithinDistance (const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers)
{
  if (indices_src_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelGeneralRegistration::selectWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    inliers.clear ();
    return;
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelGeneralRegistration::selectWithinDistance] No target dataset given!\n");
    return;
  }

  double thresh = threshold * threshold;

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
  {
    inliers.clear ();
    return;
  }

  int nr_p = 0;
  inliers.resize (indices_->size ());
  error_sqr_dists_.resize (indices_->size ());

  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  for (size_t i = 0; i < indices_->size (); ++i)
  {
    int p0 = (*indices_src_)[i];
    int p1 = (*indices_tgt_)[i];
    Eigen::Vector4f pt_src (input_->points[p0].x,
                            input_->points[p0].y,
                            input_->points[p0].z, 1);
    Eigen::Vector4f pt_tgt (target_->points[p1].x,
                            target_->points[p1].y,
                            target_->points[p1].z, 1);

    Eigen::Vector4f p_tr (transform * pt_src);

    float distance = (p_tr - pt_tgt).squaredNorm ();
    // Calculate the distance from the transformed point to its correspondence
    if (distance < thresh)
    {
      inliers[nr_p] = i;
      error_sqr_dists_[nr_p] = static_cast<double> (distance);
      ++nr_p;
    }
  }
  inliers.resize (nr_p);
  error_sqr_dists_.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::SampleConsensusModelGeneralRegistration<PointT>::countWithinDistance (
  const Eigen::VectorXf &model_coefficients, const double threshold)
{
  if (indices_src_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelGeneralRegistration::countWithinDistance] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    return (0);
  }
  if (!target_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelGeneralRegistration::countWithinDistance] No target dataset given!\n");
    return (0);
  }

  double thresh = threshold * threshold;

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients))
    return (0);

  Eigen::Matrix4f transform;
  transform.row (0).matrix () = model_coefficients.segment<4>(0);
  transform.row (1).matrix () = model_coefficients.segment<4>(4);
  transform.row (2).matrix () = model_coefficients.segment<4>(8);
  transform.row (3).matrix () = model_coefficients.segment<4>(12);

  int nr_p = 0;
  for (size_t i = 0; i < indices_->size (); ++i)
  {
    int p0 = (*indices_src_)[i];
    int p1 = (*indices_tgt_)[i];
    Eigen::Vector4f pt_src (input_->points[p0].x,
                            input_->points[p0].y,
                            input_->points[p0].z, 1);
    Eigen::Vector4f pt_tgt (target_->points[p1].x,
                            target_->points[p1].y,
                            target_->points[p1].z, 1);

    Eigen::Vector4f p_tr (transform * pt_src);
    double d = (p_tr - pt_tgt).squaredNorm ();
    if(d < thresh)
      ++nr_p;
  }
  return (nr_p);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelGeneralRegistration<PointT>::optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients)
{
  if (indices_src_->size () != indices_tgt_->size ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelGeneralRegistration::optimizeModelCoefficients] Number of source indices (%lu) differs than number of target indices (%lu)!\n", indices_->size (), indices_tgt_->size ());
    optimized_coefficients = model_coefficients;
    return;
  }

  // Check if the model is valid given the user constraints
  if (!isModelValid (model_coefficients) || !target_)
  {
    optimized_coefficients = model_coefficients;
    return;
  }

  estimateRigidTransformationSVD (*input_, *target_, inliers, optimized_coefficients);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelGeneralRegistration<PointT>::estimateRigidTransformationSVD (
  const pcl::PointCloud<PointT> &cloud_src,
  const pcl::PointCloud<PointT> &cloud_tgt,
  const std::vector<int> &samples,
  Eigen::VectorXf &transform)
{
  transform.resize (16);

  Eigen::Matrix<double, 3, Eigen::Dynamic> src (3, samples.size ());
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt (3, samples.size ());

  for (size_t i = 0; i < samples.size (); ++i)
  {
    size_t idx_src = (*indices_src_)[samples[i]];
    size_t idx_tgt = (*indices_tgt_)[samples[i]];

    src (0, i) = cloud_src[idx_src].x;
    src (1, i) = cloud_src[idx_src].y;
    src (2, i) = cloud_src[idx_src].z;

    tgt (0, i) = cloud_tgt[idx_tgt].x;
    tgt (1, i) = cloud_tgt[idx_tgt].y;
    tgt (2, i) = cloud_tgt[idx_tgt].z;
  }

  // Call Umeyama directly from Eigen
  Eigen::Matrix4d transformation_matrix = pcl::umeyama (src, tgt, false);

  // Return the correct transformation
  transform.segment<4> (0).matrix () = transformation_matrix.cast<float> ().row (0);
  transform.segment<4> (4).matrix () = transformation_matrix.cast<float> ().row (1);
  transform.segment<4> (8).matrix () = transformation_matrix.cast<float> ().row (2);
  transform.segment<4> (12).matrix () = transformation_matrix.cast<float> ().row (3);
}

#define PCL_INSTANTIATE_SampleConsensusModelGeneralRegistration(T) template class PCL_EXPORTS pcl::SampleConsensusModelGeneralRegistration<T>;

#endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_REGISTRATION_H_

