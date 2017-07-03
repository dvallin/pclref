#ifndef PCL_GENERAL_PCA_H
#define PCL_GENERAL_PCA_H

#include <PclRefLIB.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/impl/pcl_base.hpp>

namespace pcl
{
  template <typename PointT>
  class GeneralPCA : public pcl::PCLBase <PointT>
  {
  public:
    typedef pcl::PCLBase <PointT> Base;
    typedef typename pcl::PointCloud<PointT> PointCloud;

    using Base::input_;
    using Base::indices_;
    using Base::initCompute;
    using Base::setInputCloud;

    /** Updating method flag */
    enum FLAG
    {
      /** keep the new basis vectors if possible */
      increase,
      /** preserve subspace dimension */
      preserve
    };

    /** \brief Default Constructor
      * \param basis_only flag to compute only the PCA basis
      */
    GeneralPCA (bool basis_only = false)
      : Base ()
      , compute_done_ (false)
      , basis_only_ (basis_only)
      , eigenvectors_ ()
      , coefficients_ ()
      , mean_ ()
      , eigenvalues_  ()
      , inv_eigenvalues_  ()
      , eigenvalues_matrix_  ()
    {
      project_dimension_ = pclref::pclDataLength<PointT>();
    }

    GeneralPCA (const PointCloud &cloud, bool basis_only);

    /** Copy Constructor
      * \param[in] pca PCA object
      */
    GeneralPCA (const GeneralPCA& pca)
      : Base (pca)
      , compute_done_ (pca.compute_done_)
      , basis_only_ (pca.basis_only_)
      , eigenvectors_ (pca.eigenvectors_)
      , coefficients_ (pca.coefficients_)
      , mean_ (pca.mean_)
      , eigenvalues_  (pca.eigenvalues_)
      , inv_eigenvalues_ (pca.inv_eigenvalues_)
      , eigenvalues_matrix_ (pca.eigenvalues_matrix_)
      , project_dimension_ (pca.project_dimension_)
    {}

    /** Copy Constructor
      * \param[in] pca PCA object
      */
    GeneralPCA (const Eigen::MatrixXd& eigenvectors, const Eigen::VectorXd mean,
                const Eigen::MatrixXd& eigenvalues)
      : Base ()
      , compute_done_ (true)
      , basis_only_ (true)
      , eigenvectors_ (eigenvectors)
      , coefficients_ ()
      , mean_ (mean)
      , eigenvalues_  (eigenvalues)
    {
      project_dimension_ = pclref::pclDataLength<PointT>();
      inv_eigenvalues_ = Eigen::MatrixXd::Zero(eigenvectors_.rows(), eigenvectors_.rows());
      eigenvalues_matrix_ = Eigen::MatrixXd::Zero(eigenvectors_.rows(), eigenvectors_.rows());
      for(size_t i = 0; i < eigenvectors_.rows(); ++i)
      {
        // note that negative eigenvalues indicate a model misspecification,
        // if low in magnitude it typically indicates too many highly correlated
        // variables, else it should be a more serious issue!
        float ev = eigenvalues_(i) > 0 ? eigenvalues_(i) : 0;
        eigenvalues_matrix_(i,i) = std::sqrt(ev + 0.0001f);
        inv_eigenvalues_(i,i) = (1.0f / eigenvalues_matrix_(i,i));
      }
    }

    /** Assignment operator
      * \param[in] pca PCA object
      */
    inline GeneralPCA&
    operator= (GeneralPCA const & pca)
    {
      project_dimension_ = pca.project_dimension_;
      eigenvectors_ = pca.eigenvectors_;
      coefficients_ = pca.coefficients_;
      eigenvalues_  = pca.eigenvalues_;
      mean_         = pca.mean_;
      inv_eigenvalues_ = pca.inv_eigenvalues_;
      eigenvalues_matrix_ = pca.eigenvalues_matrix_;
      compute_done_ = pca.compute_done_;
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

    /** \brief Mean accessor
      * \throw InitFailedException
      */
    inline Eigen::VectorXd&
    getMean ()
    {
      if (!compute_done_)
        compute ();
      if (!compute_done_)
        PCL_THROW_EXCEPTION (InitFailedException,
                             "[pcl::PCA::getMean] PCA initCompute failed");
      return (mean_);
    }

    /** Eigen Vectors accessor
      * \throw InitFailedException
      */
    inline Eigen::MatrixXd&
    getEigenVectors ()
    {
      if (!compute_done_)
        compute ();
      if (!compute_done_)
        PCL_THROW_EXCEPTION (InitFailedException,
                             "[pcl::PCA::getEigenVectors] PCA initCompute failed");
      return (eigenvectors_);
    }

    /** Eigen Values accessor
      * \throw InitFailedException
      */
    inline Eigen::VectorXd&
    getEigenValues ()
    {
      if (!compute_done_)
        compute ();
      if (!compute_done_)
        PCL_THROW_EXCEPTION (InitFailedException,
                             "[pcl::PCA::getEigenVectors] PCA getEigenValues failed");
      return (eigenvalues_);
    }

    /** Coefficients accessor
      * \throw InitFailedException
      */
    inline Eigen::MatrixXd&
    getCoefficients ()
    {
      if (!compute_done_)
        compute ();
      if (!compute_done_)
        PCL_THROW_EXCEPTION (InitFailedException,
                             "[pcl::PCA::getEigenVectors] PCA getCoefficients failed");
      return (coefficients_);
    }

    void compute();

    /** update PCA with a new point
      * \param[in] input input point
      * \param[in] flag update flag
      * \throw InitFailedException
      */
    inline void
    update (const PointT& input, FLAG flag = preserve);

    /** Project point on the eigenspace.
      * \param[in] input point from original dataset
      * \param[out] projection the point in eigen vectors space
      * \throw InitFailedException
      */
    void
    project (const PointT& input, PointT& projection, bool whitened = true, bool use_zca = false);

    /** Project cloud on the eigenspace.
      * \param[in] input cloud from original dataset
      * \param[out] projection the cloud in eigen vectors space
      * \throw InitFailedException
      */
    void
    project (const PointCloud& input, PointCloud& projection, bool whitened = true, bool use_zca = false);

    /** Reconstruct point from its projection
      * \param[in] projection point from eigenvector space
      * \param[out] input reconstructed point
      * \throw InitFailedException
      */
    void
    reconstruct (const PointT& projection, PointT& input, bool whitened = true, bool use_zca = false);

    /** Reconstruct cloud from its projection
      * \param[in] projection cloud from eigenvector space
      * \param[out] input reconstructed cloud
      * \throw InitFailedException
      */
    inline void
    reconstruct (const PointCloud& projection, PointCloud& input, bool whitened = true, bool use_zca = false);

    void setProjectionDimension(size_t dim);

  private:
    bool initCompute ();

    bool compute_done_;
    bool basis_only_;
    size_t project_dimension_;
    Eigen::MatrixXd eigenvectors_;
    Eigen::MatrixXd inv_eigenvalues_;
    Eigen::MatrixXd eigenvalues_matrix_;
    Eigen::MatrixXd coefficients_;
    Eigen::VectorXd mean_;
    Eigen::VectorXd eigenvalues_;

  }; // class PCA
} // namespace pcl
#include <general_pca.hpp>

#endif

