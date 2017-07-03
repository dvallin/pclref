#ifndef PCLREF_LRT_H
#define PCLREF_LRT_H

#include <PclRefLIB.h>
#include <pcl/pcl_macros.h>

namespace pclref
{
  /**
  * \class LRT
  *
  * \brief Projects features using the likelihood ratio test transformation described in <a href="http://link.springer.com/chapter/10.1007/978-3-642-13408-1_18">Bosse et al. 2010</a>
  *
  */
  template <typename PointT>
  class LRT
  {
  public:
    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

    /** \brief Default Constructor
      */
    LRT ()
      : mComputeDone (false)
    {
    }

    /** Copy Constructor
      * \param[in] lrt LRT object
      */
    LRT (const LRT& lrt)
      : mComputeDone (lrt.mComputeDone)
      , mTransform (lrt.mTransform)
      , mMatchDiffs (lrt.mMatchDiffs)
      , mUnmatchDiffs (lrt.mUnmatchDiffs)
      , mTransformInv (lrt.mTransformInv)
    {}

    /** Copy Constructor
      * \param[in] transform LRT transform
      */
    LRT (const Eigen::MatrixXd& transform)
      : mComputeDone (true)
      , mTransform (transform)
    {
      mTransformInv = mTransform.inverse();
    }

    /** Assignment operator
      * \param[in] lrt LRT object
      */
    inline LRT&
    operator= (LRT const & lrt)
    {
      mComputeDone = lrt.mComputeDone;
      mMatchDiffs = lrt.mMatchDiffs;
      mUnmatchDiffs = lrt.mUnmatchDiffs;
      mTransform = lrt.mTransform;
      mTransformInv = lrt.mTransformInv;
      return (*this);
    }

    /** \brief Provide a pointer to the input source
      * (e.g., the point cloud that we want to align to the target)
      *
      * \param[in] cloud the input point cloud source
      */
    inline void
    setMatchDiffs (const PointCloudPtr &cloud);

    /** \brief Get a pointer to the input point cloud dataset target. */
    inline PointCloudPtr const
    getMatchDiffs ()
    {
      return (mMatchDiffs );
    }

    /** \brief Provide a pointer to the input target
      * (e.g., the point cloud that we want to align the input source to)
      * \param[in] cloud the input point cloud target
      */
    inline void
    setUnmatchDiffs (const PointCloudPtr &cloud);

    /** \brief Get a pointer to the input point cloud dataset target. */
    inline PointCloudPtr const
    getUnmatchDiffs ()
    {
      return (mUnmatchDiffs );
    }

    /** Transform accessor
      * \throw InitFailedException
      */
    inline Eigen::MatrixXd&
    getTransform ()
    {
      if (!mComputeDone)
        compute ();
      return (mTransform);
    }
    /** Inversre transform accessor
      * \throw InitFailedException
      */
    inline Eigen::MatrixXd&
    getTransformInv ()
    {
      if (!mComputeDone)
        compute ();
      return (mTransformInv);
    }

    void compute();

    /** Project point on the eigenspace.
      * \param[in] input point from original dataset
      * \param[out] projection the point in eigen vectors space
      * \throw InitFailedException
      */
    void
    project (const PointT& input, PointT& projection);

    /** Project cloud on the eigenspace.
      * \param[in] input cloud from original dataset
      * \param[out] projection the cloud in eigen vectors space
      * \throw InitFailedException
      */
    void
    project (const PointCloud& input, PointCloud& projection);

    /** Reconstruct point from its projection
      * \param[in] projection point from eigenvector space
      * \param[out] input reconstructed point
      * \throw InitFailedException
      */
    void
    reconstruct (const PointT& projection, PointT& input);

    /** Reconstruct cloud from its projection
      * \param[in] projection cloud from eigenvector space
      * \param[out] input reconstructed cloud
      * \throw InitFailedException
      */
    inline void
    reconstruct (const PointCloud& projection, PointCloud& input);


  private:
    bool initCompute ();

    bool mComputeDone;
    PointCloudPtr mMatchDiffs;
    PointCloudPtr mUnmatchDiffs;
    Eigen::MatrixXd mTransform;
    Eigen::MatrixXd mTransformInv;

  }; // class PCA
} // namespace pcl
#include <LRT.hpp>

#endif

