/**
 *
 * \author $Author: schuler $
 *
 * \version $Revision: 1.0 $
 *
 * \date $Date: 2015/03/11 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: May 19 2014
 *
 */

#ifndef PCLREF_LIB_H_
#define PCLREF_LIB_H_

//#define PCLREF_OPENCV
// if defined, only the most important classes are built
#define PCLREF_LEAN
// if defined, openmp will be used to improve speed
#define PCLREF_PARALLEL

/*! \mainpage The PCLref Library
 *
 * The main.cc gives a complete example application, showcasing nearly all the functionality of the PCLref Library.
 * Standard parameters should be looked up at the config folder. Command-line arguments are
 * only used to get the app running, the Parameters are what governs the actual registration and are fed via parameter file to the app.
 * The locations of the point clouds are supplied by a session file to the app. It also contains parsing information for ground truth.
 * Usage examples (although based on my local file structure) are in the tests folder, calling help gives a comprehensive list of arguments.
 *
 * Since the example application might be a bit overwhelming, good starting points are:
 * @see pclref::ProcessingStep
 * @see pclref::Combiner
 * @see pclref::PipelineFactory
 * @see pclref::Harris3DKeypoints
 * @see pclref::FPFHFeatures
 * @see pclref::EstimateCorrespondences
 * @see pclref::Ransac
 */

#include <string>
#include <iterator>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <string>
#include <algorithm>
#include <stdint.h>
#include <ctime>
#include <functional>
#include <iomanip>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/type_traits.hpp>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/preprocessor.hpp>
#include <boost/any.hpp>

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

namespace pclref
{

  // id typedefs
  typedef unsigned int identifier;
#define INVALID_ID 0u
#define FIRST_ID 1u
#define IS_VALID_ID(id) ((bool)(id > 0u))

#define NULL_PTR 0

  /// template utility function to cast types to a string
  template<typename T>
  std::string precision_cast(const T& source, int precision)
  {
    std::ostringstream ss;
    ss.exceptions(std::ios::failbit | std::ios::badbit);
    ss << std::fixed << std::setprecision(precision);
    ss << source;

    return ss.str();
  }
  /// template utility function to cast types to a string, guesses needed precision,
  /// assumes the actual value is not too obscure (i.e. 0.005 or 3.14159, not 0.100000000001)
  template<typename T>
  std::string precision_cast(const T source)
  {
    T comp = fabs(source);
    int p = 0;
    while(comp)
    {
      T d = comp - (int)comp;
      if(fabs(d) <= 0.0000001)
        break;
      T d2 = fabs((T)1.0 - d);
      if(fabs(d2) <= 0.0000001)
        break;
      comp *= 10;
      ++p;
    }
    return precision_cast(source, p);
  }

}


#ifdef PCLREF_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#endif

//#define PCL_NO_PRECOMPILE

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/for_each_product.hpp>
#include <boost/preprocessor/seq/to_tuple.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/expand.hpp>

#define PCLREF_INSTANTIATE_IMPL(r, TEMPLATE, POINT_TYPE) \
  BOOST_PP_CAT(PCLREF_INSTANTIATE_, TEMPLATE)(POINT_TYPE)

#define PCLREF_INSTANTIATE(TEMPLATE, POINT_TYPES)        \
  BOOST_PP_SEQ_FOR_EACH(PCLREF_INSTANTIATE_IMPL, TEMPLATE, POINT_TYPES)

#ifdef _MSC_VER
#define PCLREF_INSTANTIATE_PRODUCT_IMPL(r, product) \
  BOOST_PP_CAT(PCLREF_INSTANTIATE_, BOOST_PP_SEQ_HEAD(product)) \
          BOOST_PP_EXPAND(BOOST_PP_SEQ_TO_TUPLE(BOOST_PP_SEQ_TAIL(product)))
#else
#define PCLREF_INSTANTIATE_PRODUCT_IMPL(r, product) \
  BOOST_PP_EXPAND(BOOST_PP_CAT(PCLREF_INSTANTIATE_, BOOST_PP_SEQ_HEAD(product)) \
          BOOST_PP_SEQ_TO_TUPLE(BOOST_PP_SEQ_TAIL(product)))
#endif


#define PCLREF_INSTANTIATE_PRODUCT(TEMPLATE, PRODUCT) \
  BOOST_PP_SEQ_FOR_EACH_PRODUCT(PCLREF_INSTANTIATE_PRODUCT_IMPL, ((TEMPLATE))PRODUCT)

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/feature.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/transforms.h>

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<153>,
                                   (float[153], histogram, hist153)
                                  )
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<10>,
                                   (float[10], histogram, hist10)
                                  )

#include <PointTypeWrappers.h>

namespace pclref
{
  /// helper struct to copy over boost pointers without invoking the destructor
  struct null_deleter
  {
    void operator()(void const *) const
    {
    }
  };

  /// template helper to get length of pcl datatypes
  template<typename T>
  size_t pclDataLength()
  {
    return 3;
  }
  /// template helper to get the float array of pcl datatypes
  template<typename T>
  float* pclData(T& x)
  {
    return x.data;
  }
  /// template helper to get the float array of pcl datatypes
  template<typename T>
  const float* pclData(const T& x)
  {
    return x.data;
  }

// some speciallizations of the above
  template<> size_t pclDataLength<pcl::FPFHSignature33>(); ///< template helper to get length of pcl datatypes
  template<> size_t pclDataLength<pcl::Narf36>(); ///< template helper to get length of pcl datatypes
  template<> size_t pclDataLength<pcl::VFHSignature308>(); ///< template helper to get length of pcl datatypes
  template<> size_t pclDataLength<pcl::ShapeContext1980>(); ///< template helper to get length of pcl datatypes
  template<> size_t pclDataLength<pcl::SHOT352>(); ///< template helper to get length of pcl datatypes
  template<> size_t pclDataLength<pcl::ESFSignature640>(); ///< template helper to get length of pcl datatypes
  template<> size_t pclDataLength<pcl::Histogram<153> >(); ///< template helper to get length of pcl datatypes
  template<> size_t pclDataLength<pcl::Histogram<10> >(); ///< template helper to get length of pcl datatypes
  template<> const float* pclData<pcl::FPFHSignature33>(const pcl::FPFHSignature33& x); ///< template helper to get the float array of pcl datatypes
  template<> const float* pclData<pcl::Narf36>(const pcl::Narf36& x); ///< template helper to get the float array of pcl datatypes
  template<> const float* pclData<pcl::VFHSignature308>(const pcl::VFHSignature308& x); ///< template helper to get the float array of pcl datatypes
  template<> const float* pclData<pcl::ShapeContext1980>(const pcl::ShapeContext1980& x); ///< template helper to get the float array of pcl datatypes
  template<> const float* pclData<pcl::SHOT352>(const pcl::SHOT352& x); ///< template helper to get the float array of pcl datatypes
  template<> const float* pclData<pcl::ESFSignature640>(const pcl::ESFSignature640& x); ///< template helper to get the float array of pcl datatypes
  template<> const float* pclData<pcl::Histogram<153> >(const pcl::Histogram<153>& x); ///< template helper to get the float array of pcl datatypes
  template<> const float* pclData<pcl::Histogram<10> >(const pcl::Histogram<10>& x); ///< template helper to get the float array of pcl datatypes
  template<> float* pclData<pcl::FPFHSignature33>(pcl::FPFHSignature33& x); ///< template helper to get the float array of pcl datatypes
  template<> float* pclData<pcl::Narf36>(pcl::Narf36& x); ///< template helper to get the float array of pcl datatypes
  template<> float* pclData<pcl::VFHSignature308>(pcl::VFHSignature308& x); ///< template helper to get the float array of pcl datatypes
  template<> float* pclData<pcl::ShapeContext1980>(pcl::ShapeContext1980& x); ///< template helper to get the float array of pcl datatypes
  template<> float* pclData<pcl::SHOT352>(pcl::SHOT352& x); ///< template helper to get the float array of pcl datatypes
  template<> float* pclData<pcl::ESFSignature640>(pcl::ESFSignature640& x); ///< template helper to get the float array of pcl datatypes
  template<> float* pclData<pcl::Histogram<153> >(pcl::Histogram<153>& x); ///< template helper to get the float array of pcl datatypes
  template<> float* pclData<pcl::Histogram<10> >(pcl::Histogram<10>& x); ///< template helper to get the float array of pcl datatypes

  /// extracts scale from a PointType containing scale. Needs to be specialized to give something other than fallback_scale.
  template<typename K> class GetScale
  {
  public:
    GetScale(float fallback_scale, bool extract_scale = true)
      : m_fallback_scale(fallback_scale), m_extract_scale(extract_scale)
    { }

    float operator() (K&)
    {
      return m_fallback_scale;
    }

  private:
    float m_fallback_scale;
    bool m_extract_scale;
  };
  /// Specialization of GetScale for PointWithScale
  template<> class GetScale<pcl::PointWithScale>
  {
  public:
    GetScale(float fallback_scale, bool extract_scale = true)
      : m_fallback_scale(fallback_scale), m_extract_scale(extract_scale)
    { }

    float operator() (pcl::PointWithScale& k)
    {
      return m_extract_scale ? k.scale : m_fallback_scale;
    }
  private:
    float m_fallback_scale;
    bool m_extract_scale;
  };

// from pcl
// enum NormType {L1, L2_SQR, L2, LINF, JM, B, SUBLINEAR, CS, DIV, PF, K, KL, HIK};
  /// wrapper to measure distance of pcl datatypes
  template<typename T>
  double distance(const T& x, const T& y, pcl::NormType type)
  {
    return pcl::selectNorm(pclData(x), pclData(y), pclDataLength<T>(), type);
  }
  /// wrapper to test pcl datatypes
  template<typename T>
  bool isValid(const T& p)
  {
    const float* data = pclData(p);
    const float* end = data + pclDataLength<T>();
    while(data != end)
    {
      if(!pcl_isfinite(*data))
        return false;
      ++data;
    }
    return true;
  }
  /// computes the centroid of an indexed cloud.
  template<typename T>
  void computeCentroid(const pcl::PointCloud<T>& cloud,
                       const std::vector<int>& indices,
                       Eigen::VectorXd& centroid)
  {
    const size_t dataLength = pclDataLength<T>();
    centroid = Eigen::VectorXd::Zero (dataLength);
    int c = 0;
    for(size_t i = 0; i < indices.size(); ++i)
    {
      int idx = indices[i];
      if(!isValid(cloud[idx]))
        continue;
      const float* p = pclData(cloud[idx]);
      for(size_t j = 0; j < dataLength; ++j)
      {
        centroid[j] += p[j];
      }
      ++c;
    }
    for(size_t j = 0; j < dataLength; ++j)
    {
      centroid[j] /= c;
    }
  }
  /// creates a demeaned cloud based on an indexed cloud and a centroid
  template<typename T>
  void demeanCloud(const pcl::PointCloud<T>& cloud,
                   const std::vector<int>& indices,
                   const Eigen::VectorXd& centroid,
                   Eigen::MatrixXd& cloud_out)
  {
    const size_t dataLength = pclref::pclDataLength<T>();
    const size_t pointCount = indices.size();
    cloud_out = Eigen::MatrixXd::Zero (dataLength, pointCount);
    for(size_t i = 0; i < indices.size(); ++i)
    {
      int idx = indices[i];
      if(!isValid(cloud[idx]))
        continue;
      const float* p = pclData(cloud[idx]);
      int j = 0;
      for(size_t j = 0; j < dataLength; ++j)
      {
        cloud_out (j, i) = p[j] - centroid[j];
      }
    }
  }

  /// PointType to Eigen::Vector
  template<typename T>
  Eigen::VectorXd toEigenVector(const T& point)
  {
    const size_t dataLength = pclref::pclDataLength<T>();
    Eigen::VectorXd result = Eigen::VectorXd::Zero (dataLength);

    const float* p = pclData(point);
    for(size_t j = 0; j < dataLength; ++j)
    {
      result[j] = p[j];
    }
    return result;
  }
  /// Eigen::Vector4 to a PointType with XYZ
  template<typename T, typename Real>
  T EigenVector4toXYZ(const Eigen::Matrix<Real, 4, 1>& p)
  {
    T result;
    result.x = p[0];
    result.y = p[1];
    result.z = p[2];
    return result;
  }
  /// PointType with XYZ to Eigen::Vector4
  template<typename T, typename Real>
  Eigen::Matrix<Real, 4, 1> XYZtoEigenVector4(const T& p)
  {
    Eigen::Matrix<Real, 4, 1> result;
    result[0] = p.x;
    result[1] = p.y;
    result[2] = p.z;
    result[3] = 1.0;
    return result;
  }
  /// creates a PointType from Eigen::Vector
  template<typename T, typename Real>
  T fromEigenVector(const Eigen::Matrix<Real, Eigen::Dynamic, 1> v)
  {
    const size_t dataLength = pclref::pclDataLength<T>();
    T result = T();
    float* p = pclData(result);
    for(size_t j = 0; j < dataLength; ++j)
    {
      p[j] = v[j];
    }
    return result;
  }
  /// compute the Covariance Matrix of a point cloud (implicitely demeaned)
  template <typename PointT>
  unsigned int computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                        const Eigen::MatrixXd &centroid,
                                        Eigen::MatrixXd &covariance_matrix)
  {
    if (cloud.empty ())
      return (0);

    // Initialize to 0
    covariance_matrix.resize(centroid.rows(), centroid.rows());
    covariance_matrix.setZero ();

    unsigned point_count;
    // If the data is dense, we don't need to check for NaN
    if (cloud.is_dense)
    {
      point_count = static_cast<unsigned> (cloud.size ());
      // For each point in the cloud
      for (size_t i = 0; i < point_count; ++i)
      {
        Eigen::VectorXd pt = toEigenVector(cloud[i]) - centroid;

        for(size_t r = 0; r < covariance_matrix.rows(); ++r)
        {
          for(size_t c = 0; c < covariance_matrix.cols(); ++c)
          {
            covariance_matrix (r, c) += pt(r) * pt(c);
          }
        }
      }
    }
    // NaN or Inf values could exist => check for them
    else
    {
      point_count = 0;
      // For each point in the cloud
      for (size_t i = 0; i < cloud.size (); ++i)
      {
        // Check if the point is invalid
        if (!isValid (cloud [i]))
          continue;

        Eigen::VectorXd pt = toEigenVector(cloud[i]) - centroid;

        for(size_t r = 0; r < covariance_matrix.rows(); ++r)
        {
          for(size_t c = 0; c <= r; ++c)
          {
            covariance_matrix (r, c) += pt(r) * pt(c);
          }
        }

        ++point_count;
      }
    }
    for(size_t r = 0; r < covariance_matrix.rows(); ++r)
    {
      for(size_t c = r; c < covariance_matrix.rows(); ++c)
      {
        if(r != c)
          covariance_matrix (r, c) = covariance_matrix (c, r);
      }
    }
    return (point_count);
  }
  //////////////////////////////////////////////////////////////////////////////////////////////
  /// compute the Normalized Covariance Matrix of a point cloud (implicitely demeaned)
  template <typename PointT>
  unsigned int computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud,
      const Eigen::MatrixXd &centroid,
      Eigen::MatrixXd &covariance_matrix)
  {
    unsigned point_count = computeCovarianceMatrix (cloud, centroid, covariance_matrix);
    if (point_count != 0)
      covariance_matrix /= static_cast<float> (point_count-1);

    return (point_count);
  }
}

#endif /* PCLREF_LIB_H_ */
