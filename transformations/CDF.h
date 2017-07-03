#ifndef PCLREF_CDF_H
#define PCLREF_CDF_H

#include <PclRefLIB.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>

namespace pclref
{
  /**
  * \class CDF
  *
  * \brief Projects features component-wise using a inverse cumulative distribution function.
  *
  * If no matrix is given that encodes the Distribution to be used for each component of the
  * feature vector, one is generated based on the actual distribution of the given feature cloud.
  * This leads to a non-linear transformation to the uniform distribution.
  *
  * \note This class only interpolates linearly between samples!
  */
  template <typename PointT>
  class CDF : public pcl::PCLBase <PointT>
  {
  public:
    typedef pcl::PCLBase <PointT> Base; ///< type of the base class
    typedef typename pcl::PointCloud<PointT> PointCloud; /// type of a point cloud

    using Base::input_;
    using Base::indices_;
    using Base::initCompute;
    using Base::setInputCloud;

    /** \brief Default Constructor
      * \param samples Amount of samples to be used to generate the CDF transform
      */
    CDF (float samples)
      : Base ()
      , compute_done_ (false)
      , cdf_  ()
      , matrix_ ()
      , samples_ (samples)
    {
    }

    CDF (const PointCloud &cloud);

    /** Copy Constructor
      * \param[in] cdf CDF object
      */
    CDF (const CDF& cdf)
      : Base (cdf)
      , compute_done_ (cdf.compute_done_)
      , cdf_ ()
      , matrix_ (cdf.matrix_)
      , samples_ (cdf.samples_)
    {
      cdf_.resize(cdf.cdf_.size());
      for(size_t i = 0; i < cdf_.size(); ++i)
        std::copy(cdf.cdf_[i].begin(), cdf.cdf_[i].end(), cdf_[i].begin());
    }

    /** Copy Constructor
      * \param[in] cdf_matrix A matrix encoding the distribution for each component of the feature.
      */
    CDF (const Eigen::MatrixXf& cdf_matrix)
      : Base ()
      , compute_done_ (true)
      , matrix_  (cdf_matrix)
      , samples_ ()
    {
      cdf_.resize(cdf_matrix.cols());
      samples_ = cdf_matrix.rows();
      for(size_t i = 0; i < cdf_matrix.cols(); ++i)
      {
        for(size_t j = 0; j < cdf_matrix.rows(); ++j)
        {
          cdf_[i].push_back(cdf_matrix(j,i));
        }
      }
    }

    /** Assignment operator
      * \param[in] cdf CDF object
      */
    inline CDF&
    operator= (CDF const & cdf)
    {
      matrix_ = cdf.matrix_;
      cdf_.resize(cdf.cdf_.size());
      for(size_t i = 0; i < cdf_.size(); ++i)
        std::copy(cdf.cdf_[i].begin(), cdf.cdf_[i].end(), cdf_[i].begin());
      samples_ = cdf.samples_;
      return (*this);
    }

    /** \brief Provide a pointer to the input dataset
      * \param cloud the const boost shared pointer to a PointCloud message
      */
    inline void
    setInputCloud (const typename PointCloud::Ptr &cloud)
    {
      Base::setInputCloud (cloud);
      compute_done_ = false;
    }

    /// Returns a matrix encoding the distribution for each component of the feature.
    inline Eigen::MatrixXf&
    getMatrix ()
    {
      if (!compute_done_)
        compute ();

      return (matrix_);
    }

    void compute();

    /** Project point.
      * \param[in] input point from original dataset
      * \param[out] projection the point in the transformed space
      */
    void
    project (const PointT& input, PointT& projection);

    /** Project cloud.
      * \param[in] input cloud from original dataset
      * \param[out] projection the cloud in the transformed space
      */
    void
    project (const PointCloud& input, PointCloud& projection);

    /** Reconstruct point from its projection
      * \param[in] projection point in the transformed space
      * \param[out] input reconstructed point
      */
    void
    reconstruct (const PointT& projection, PointT& input);

    /** Reconstruct cloud from its projection
      * \param[in] projection cloud in the transformed space
      * \param[out] input reconstructed cloud
      */
    inline void
    reconstruct (const PointCloud& projection, PointCloud& input);

  private:
    bool initCompute ();

    bool compute_done_;
    std::vector<std::vector<float> > cdf_;
    Eigen::MatrixXf matrix_;
    float samples_;

  }; // class PCA
} // namespace pcl
#include <CDF.hpp>

#endif

