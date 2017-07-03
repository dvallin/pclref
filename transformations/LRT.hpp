#ifndef PCLREF_LRT_HPP_
#define PCLREF_LRT_HPP_
#include <LRT.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/exceptions.h>
#include <Arrays.h>

namespace pclref
{
/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  LRT<PointT>::compute ()
  {
    Eigen::VectorXd uMean, mMean;
    {
      std::vector<int> indices;
      Arrays::natural(0, mUnmatchDiffs->size(), indices);
      pclref::computeCentroid(*mUnmatchDiffs, indices, uMean);
      indices.clear();
      Arrays::natural(0, mMatchDiffs->size(), indices);
      pclref::computeCentroid(*mMatchDiffs, indices, mMean);
    }
    Eigen::MatrixXd uCov, mCov;
    {
      pclref::computeCovarianceMatrixNormalized(*mUnmatchDiffs, uMean, uCov);
      pclref::computeCovarianceMatrixNormalized(*mMatchDiffs, mMean, mCov);
    }
    Eigen::MatrixXd A = mCov.inverse() - uCov.inverse();
    Eigen::LDLT<Eigen::MatrixXd> chol(A);

    mTransform = chol.matrixU();
    // in the (not so rare) case A is not psd, there exist negative eigenvalues,
    // s.t. #negative eigenvalues = #negative diagonal entries. calc sqrt(D) with this
    // in mind
    Eigen::VectorXd ds = chol.vectorD();
    for(size_t i = 0; i < ds.rows(); ++i)
      mTransform(i,i) = ds(i) > 0 ? sqrt(ds(i)) : 0;
    mTransformInv = mTransform.inverse();
  }
/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  LRT<PointT>::setMatchDiffs (const PointCloudPtr &cloud)
  {
    mMatchDiffs = cloud;
  }
/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  LRT<PointT>::setUnmatchDiffs (const PointCloudPtr &cloud)
  {
    mUnmatchDiffs = cloud;
  }
/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  LRT<PointT>::project (const PointT& input, PointT& projection)
  {
    if(!mComputeDone)
      compute ();

    Eigen::VectorXd proj = pclref::toEigenVector(input);
    proj = mTransform * proj;
    projection = pclref::fromEigenVector<PointT>(proj);
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  LRT<PointT>::project (const PointCloud& input, PointCloud& projection)
  {
    if(!mComputeDone)
      compute ();

    if (input.is_dense)
    {
      projection.resize (input.size ());
      for (size_t i = 0; i < input.size (); ++i)
        project (input[i], projection[i]);
    }
    else
    {
      PointT p;
      for (size_t i = 0; i < input.size (); ++i)
      {
        if (!pclref::isValid(input[i]))
          continue;
        project (input[i], p);
        projection.push_back (p);
      }
    }
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  LRT<PointT>::reconstruct (const PointT& projection, PointT& input)
  {
    if(!mComputeDone)
      compute ();

    Eigen::VectorXd proj = pclref::toEigenVector(projection);
    proj = mTransformInv * proj;
    input = pclref::fromEigenVector<PointT>(proj);
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  LRT<PointT>::reconstruct (const PointCloud& projection, PointCloud& input)
  {
    if(!mComputeDone)
      compute ();
    if (input.is_dense)
    {
      input.resize (projection.size ());
      for (size_t i = 0; i < projection.size (); ++i)
        reconstruct (projection[i], input[i]);
    }
    else
    {
      PointT p;
      for (size_t i = 0; i < input.size (); ++i)
      {
        if (!pclref::isValid(input[i]))
          continue;
        reconstruct (projection[i], p);
        input.push_back (p);
      }
    }
  }
}
#endif
