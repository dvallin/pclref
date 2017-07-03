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
#ifndef PCL_GENERAL_PCA_HPP_
#define PCL_GENERAL_PCA_HPP_
#include <general_pca.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/exceptions.h>

namespace pcl
{
/////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Constructor with direct computation
    * \param[in] cloud input m*n matrix (ie n vectors of R(m))
    * \param[in] basis_only flag to compute only the PCA basis
    */
  template<typename PointT>
  GeneralPCA<PointT>::GeneralPCA (const PointCloud &cloud, bool basis_only)
  {
    Base ();
    basis_only_ = basis_only;
    setInputCloud (cloud.makeShared ());
  }

  template<typename PointT> void
  GeneralPCA<PointT>::compute ()
  {
    if (!compute_done_)
      initCompute ();
    else return; // use update instead

    size_t dataLength = pclref::pclDataLength<PointT>();

    eigenvalues_.resize(dataLength);
    eigenvectors_.resize(dataLength, dataLength);

    // Compute mean
    pclref::computeCentroid (*input_, *indices_, mean_);

    // Compute demeanished cloud
    Eigen::MatrixXd cloud_demean;
    pclref::demeanCloud (*input_, *indices_, mean_, cloud_demean);
    assert (cloud_demean.cols () == int (indices_->size ()));

    // Compute the product cloud_demean * cloud_demean^T
    Eigen::MatrixXd alpha = static_cast<Eigen::MatrixXd> (cloud_demean * cloud_demean.transpose ());

    // Compute eigen vectors and values
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> evd (alpha);
    // Organize eigenvectors and eigenvalues in ascendent order
    for (int i = 0; i < dataLength; ++i)
    {
      eigenvalues_[i] = evd.eigenvalues () [dataLength-i-1];
      eigenvectors_.col (i) = evd.eigenvectors ().col (dataLength-i-1);
    }

    inv_eigenvalues_ = Eigen::MatrixXd::Zero(eigenvectors_.rows(), eigenvectors_.rows());
    eigenvalues_matrix_ = Eigen::MatrixXd::Zero(eigenvectors_.rows(), eigenvectors_.rows());
    for(size_t i = 0; i < eigenvectors_.rows(); ++i)
    {
      float ev = eigenvalues_(i) > 0 ? eigenvalues_(i) : 0;
      eigenvalues_matrix_(i,i) = std::sqrt(ev + 0.0001f);
      inv_eigenvalues_(i,i) = (1.0f / eigenvalues_matrix_(i,i));
    }

    // If not basis only then compute the coefficients
    if (!basis_only_)
      coefficients_ = eigenvectors_.transpose() * cloud_demean;
    compute_done_ = true;
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> bool
  GeneralPCA<PointT>::initCompute ()
  {
    if(!Base::initCompute ())
    {
      PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::initCompute] failed");
      return (false);
    }
    if(indices_->size () < 3)
    {
      PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::initCompute] number of points < 3");
      return (false);
    }
    return (true);
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> inline void
  GeneralPCA<PointT>::update (const PointT& input_point, FLAG flag)
  {
    if (!compute_done_)
      initCompute ();
    if (!compute_done_)
      PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::update] PCA initCompute failed");

    Eigen::VectorXd input = pclref::toEigenVector(input_point);
    const size_t n = eigenvectors_.cols ();// number of eigen vectors
    Eigen::VectorXd meanp = (float(n) * (mean_ + input)) / float(n + 1);
    Eigen::VectorXd a = eigenvectors_.transpose() * (input - mean_);
    Eigen::VectorXd y = (eigenvectors_ * a) + mean_;
    Eigen::VectorXd h = y - input;
    if (h.norm() > 0)
      h.normalize ();
    else
      h.setZero ();
    float gamma = h.dot(input - mean_);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero (a.size() + 1, a.size() + 1);
    D.block(0,0,n,n) = a * a.transpose();
    D /=  float(n)/float((n+1) * (n+1));
    for(std::size_t i=0; i < a.size(); i++)
    {
      D(i,i)+= float(n)/float(n+1)*eigenvalues_(i);
      D(D.rows()-1,i) = float(n) / float((n+1) * (n+1)) * gamma * a(i);
      D(i,D.cols()-1) = D(D.rows()-1,i);
      D(D.rows()-1,D.cols()-1) = float(n)/float((n+1) * (n+1)) * gamma * gamma;
    }

    Eigen::MatrixXd R(D.rows(), D.cols());
    Eigen::EigenSolver<Eigen::MatrixXd> D_evd (D, false);
    Eigen::VectorXd alphap = D_evd.eigenvalues().real();
    eigenvalues_.resize(eigenvalues_.size() +1);
    for(std::size_t i=0; i<eigenvalues_.size(); i++)
    {
      eigenvalues_(i) = alphap(eigenvalues_.size()-i-1);
      R.col(i) = D.col(D.cols()-i-1);
    }
    Eigen::MatrixXd Up = Eigen::MatrixXd::Zero(eigenvectors_.rows(), eigenvectors_.cols()+1);
    Up.topLeftCorner(eigenvectors_.rows(),eigenvectors_.cols()) = eigenvectors_;
    Up.rightCols<1>() = h;
    eigenvectors_ = Up*R;
    if (!basis_only_)
    {
      Eigen::VectorXd etha = Up.transpose() * (mean_ - meanp);
      coefficients_.resize(coefficients_.rows()+1,coefficients_.cols()+1);
      for(std::size_t i=0; i < (coefficients_.cols() - 1); i++)
      {
        coefficients_(coefficients_.rows()-1,i) = 0;
        coefficients_.col(i) = (R.transpose() * coefficients_.col(i)) + etha;
      }
      a.resize(a.size()+1);
      a(a.size()-1) = 0;
      coefficients_.col(coefficients_.cols()-1) = (R.transpose() * a) + etha;
    }
    mean_ = meanp;
    switch (flag)
    {
    case increase:
      if (eigenvectors_.rows() >= eigenvectors_.cols())
        break;
    case preserve:
      if (!basis_only_)
        coefficients_ = coefficients_.topRows(coefficients_.rows() - 1);
      eigenvectors_ = eigenvectors_.leftCols(eigenvectors_.cols() - 1);
      eigenvalues_.resize(eigenvalues_.size()-1);
      inv_eigenvalues_ = Eigen::MatrixXd::Zero(eigenvectors_.rows(), eigenvectors_.rows());
      eigenvalues_matrix_ = Eigen::MatrixXd::Zero(eigenvectors_.rows(), eigenvectors_.rows());
      for(size_t i = 0; i < eigenvectors_.rows(); ++i)
      {
        float ev = eigenvalues_(i) > 0 ? eigenvalues_(i) : 0;
        eigenvalues_matrix_(i,i) = std::sqrt(ev + 0.0001f);
        inv_eigenvalues_(i,i) = (1.0f / eigenvalues_matrix_(i,i));
      }
      break;
    default:
      PCL_ERROR("[pcl::PCA] unknown flag\n");
    }
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  GeneralPCA<PointT>::project (const PointT& input, PointT& projection, bool whitened, bool use_zca)
  {
    if(!compute_done_)
      compute ();
    if (!compute_done_)
      PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::project] PCA initCompute failed");

    Eigen::VectorXd proj = pclref::toEigenVector(input) - mean_;
    proj = eigenvectors_.transpose() * proj;

    // easy peasy dim reduction
    for(size_t i = project_dimension_; i < pclref::pclDataLength<PointT>(); ++i)
      proj(i) = 0.f;

    if(whitened)
      proj = inv_eigenvalues_ * proj;
    if(use_zca)
      proj = eigenvectors_ * proj;

    projection = pclref::fromEigenVector<PointT>(proj);
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  GeneralPCA<PointT>::project (const PointCloud& input, PointCloud& projection, bool whitened, bool use_zca)
  {
    if(!compute_done_)
      compute ();
    if (!compute_done_)
      PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::project] PCA initCompute failed");
    if (input.is_dense)
    {
      projection.resize (input.size ());
      for (size_t i = 0; i < input.size (); ++i)
        project (input[i], projection[i], whitened, use_zca);
    }
    else
    {
      PointT p;
      for (size_t i = 0; i < input.size (); ++i)
      {
        if (!pclref::isValid(input[i]))
          continue;
        project (input[i], p, whitened, use_zca);
        projection.push_back (p);
      }
    }
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  GeneralPCA<PointT>::reconstruct (const PointT& projection, PointT& input, bool whitened, bool use_zca)
  {
    if(!compute_done_)
      compute ();
    if (!compute_done_)
      PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::reconstruct] PCA initCompute failed");

    Eigen::VectorXd proj = pclref::toEigenVector(projection);
    if(use_zca)
      proj = eigenvectors_.transpose() * proj;
    if(whitened)
      proj = eigenvalues_matrix_ * proj;

    input = pclref::fromEigenVector<PointT>(eigenvectors_ * proj + mean_);
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  GeneralPCA<PointT>::reconstruct (const PointCloud& projection, PointCloud& input, bool whitened, bool use_zca)
  {
    if(!compute_done_)
      compute ();
    if (!compute_done_)
      PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::reconstruct] PCA initCompute failed");
    if (input.is_dense)
    {
      input.resize (projection.size ());
      for (size_t i = 0; i < projection.size (); ++i)
        reconstruct (projection[i], input[i], whitened, use_zca);
    }
    else
    {
      PointT p;
      for (size_t i = 0; i < input.size (); ++i)
      {
        if (!pclref::isValid(input[i]))
          continue;
        reconstruct (projection[i], p, whitened, use_zca);
        input.push_back (p);
      }
    }
  }
/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  GeneralPCA<PointT>::setProjectionDimension(size_t dim)
  {
    project_dimension_ = dim;
  }
}
#endif
