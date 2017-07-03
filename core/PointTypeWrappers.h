#ifndef POINT_TYPE_WRAPPERS_H_
#define POINT_TYPE_WRAPPERS_H_

#include <PclRefLIB.h>
#include <pcl/point_types.h>
namespace pclref
{
  /// Wraps the Feature Type to be used
  struct FPFH_Type
  {
    typedef pcl::FPFHSignature33 Type; ///< IMPL actual pcl type
  };
  /// Wraps the Feature Type to be used
  struct NARFF_Type
  {
    typedef pcl::Narf36 Type; ///< IMPL actual pcl type
  };
  /// Wraps the Feature Type to be used
  struct SI_Type
  {
    typedef pcl::Histogram<153> Type; ///< IMPL actual pcl type
  };
  /// Wraps the Feature Type to be used
  struct SHOT_Type
  {
    typedef pcl::SHOT352 Type; ///< IMPL actual pcl type
  };
#ifndef PCLREF_LEAN
  /// Wraps the Feature Type to be used
  struct VFH_Type
  {
    typedef pcl::VFHSignature308 Type; ///< IMPL actual pcl type
  };
  /// Wraps the Feature Type to be used
  struct CVFH_Type
  {
    typedef pcl::VFHSignature308 Type; ///< IMPL actual pcl type
  };
  /// Wraps the Feature Type to be used
  struct OURCVFH_Type
  {
    typedef pcl::VFHSignature308 Type; ///< IMPL actual pcl type
  };
  /// Wraps the Feature Type to be used
  struct USC_Type
  {
    typedef pcl::ShapeContext1980 Type; ///< IMPL actual pcl type
  };
  /// Wraps the Feature Type to be used
  struct SC3D_Type
  {
    typedef pcl::ShapeContext1980 Type; ///< IMPL actual pcl type
  };
  /// Wraps the Feature Type to be used
  struct ESF_Type
  {
    typedef pcl::ESFSignature640 Type; ///< IMPL actual pcl type
  };
#endif
}
#define PCLREF_POINT_TYPES (pcl::PointXYZ)(pcl::PointWithRange)
#define PCLREF_KEYPOINT_TYPES (pcl::PointXYZ)(pcl::PointXYZI)(pcl::PointWithScale)

#ifndef PCLREF_LEAN
#define PCLREF_FEATURE_TYPES (pclref::FPFH_Type)(pclref::NARFF_Type)(pclref::ESF_Type)(pclref::VFH_Type)(pclref::CVFH_Type)(pclref::OURCVFH_Type)(pclref::USC_Type)(pclref::SI_Type)(pclref::SC3D_Type)(pclref::SHOT_Type)
#else
#define PCLREF_FEATURE_TYPES (pclref::FPFH_Type)(pclref::NARFF_Type)(pclref::SHOT_Type)(pclref::SI_Type)
#endif

#define PCLREF_TYPES_PRODUCT (PCLREF_POINT_TYPES)(PCLREF_KEYPOINT_TYPES)(PCLREF_FEATURE_TYPES)
#define PCLREF_TYPES ((pclref::FPFH_Type)(pclref::NARFF_Type)(pclref::SHOT_Type)(pclref::SI_Type)(pcl::PointXYZ)(pcl::PointWithRange)(pcl::PointXYZI))

#endif
