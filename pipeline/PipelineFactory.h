/**
 * \author $Author: schuler $
 *
 * \version $Revision: 0.1 $
 *
 * \date $Date: 2012/07/24 $
 *
 * Contact: schuler.maximilian@gmail.com
 *
 * Created on: Sep 18 2014
 *
 */

#ifndef PIPELINE_FACTORY_H_
#define PIPELINE_FACTORY_H_

// core
#include <PclRefLIB.h>
#include <DataAccessor.h>
#include <ProcessingStep.h>
#include <Combiner.h>
#include <ProcessingContext.h>
#include <ParameterContext.h>

// combiner is used as the pipeline
#include <Combiner.h>

// point cloud processing and utilities
#include <EstimateNormals.h>
#include <LoadCloud.h>
#include <LoadGroundTruth.h>
#include <TransformCloud.h>
#include <DownsampleCloud.h>
#include <ConvertToRangeImage.h>
#include <CreateKeypointIndices.h>
#include <CreateKeypointsFromIndices.h>
#include <Visualizer.h>
#include <KFInfo.h>
#include <TInfo.h>

// quality measures
#include <CalculateOverlap.h>
#include <Repeatability.h>
#include <Uniqueness.h>
#include <UniquenessSelf.h>
#include <SampleMU.h>
#include <CorrespondenceInlierRate.h>
#include <KeypointStatsSelf.h>
#include <PointCloudStats.h>
#include <Persistence.h>

// keypoints and clustering
#include <NARFKeypoints.h>
#include <ISSKeypoints.h>
#include <Harris3DKeypoints.h>
#include <UniformKeypoints.h>
#include <GrowingRegions.h>
#include <SSKeypoints.h>
#include <SIFTKeypoints.h>
#include <RandomKeypoints.h>
#include <KeypointClustering.h>
#include <KeypointNeighborhood.h>

// features
#include <NARFFeatures.h>
#include <FPFHFeatures.h>
#include <ESFFeatures.h>
#include <VFHFeatures.h>
#include <USCFeatures.h>
#include <CVFHFeatures.h>
#include <SHOTFeatures.h>
#include <SIFeatures.h>
#include <OURCVFHFeatures.h>
#include <SC3DFeatures.h>

// transformation and correspondence estimation
#include <EstimateCorrespondences.h>
#include <FeaturePCA.h>
#include <CDFTransform.h>
#include <LRTTransform.h>
#include <RemapCloud.h>
#include <RemoveRemap.h>
#include <PredictCorrespondences.h>
#include <GTCorrespondences.h>
#include <RANSAC.h>
#include <MST.h>
#include <ICP.h>
#include <NDT.h>

namespace pclref
{
  namespace factoryMeta
  {
    /// Point Type Meta structure for use with PipelineFactory
    struct XYZ
    {
      typedef pcl::PointXYZ Type; ///< IMPL Type
    };
    /// Point Type Meta structure for use with PipelineFactory
    struct PWR
    {
      typedef pcl::PointWithRange Type; ///< IMPL Type
    };
    /// Point Type Meta structure for use with PipelineFactory
    struct XYZI
    {
      typedef pcl::PointXYZI Type; ///< IMPL Type
    };

    /// Keypoint Meta structure for use with PipelineFactory
    struct Harris
    {
      typedef pcl::PointXYZI Type; ///< IMPL Type
    };
    /// Keypoint Meta structure for use with PipelineFactory
    struct NARFK
    {
      typedef pcl::PointXYZI Type; ///< IMPL Type
    };
    /// Keypoint Meta structure for use with PipelineFactory
    struct ISS
    {
      typedef pcl::PointXYZ Type; ///< IMPL Type
    };
    /// Keypoint Meta structure for use with PipelineFactory
    struct UniformK
    {
      typedef pcl::PointXYZI Type; ///< IMPL Type
    };
    /// Keypoint Meta structure for use with PipelineFactory
    struct NoKeypoints
    {
      typedef pcl::PointXYZI Type; ///< IMPL Type
    };
#ifndef PCLREF_LEAN
    /// Keypoint Meta structure for use with PipelineFactory
    struct Region
    {
      typedef pcl::PointXYZI Type; ///< IMPL Type
    };
    /// Keypoint Meta structure for use with PipelineFactory
    struct SIFT
    {
      typedef pcl::PointWithScale Type; ///< IMPL Type
    };
    /// Keypoint Meta structure for use with PipelineFactory
    struct RandomK
    {
      typedef pcl::PointXYZI Type; ///< IMPL Type
    };
    /// Keypoint Meta structure for use with PipelineFactory
    struct SS
    {
      typedef pcl::PointXYZ Type; ///< IMPL Type
    };
#endif

    /// Feature Meta structure for use with PipelineFactory
    struct FPFH
    {
      typedef FPFH_Type Type; ///< IMPL Type
    };
    /// Feature Meta structure for use with PipelineFactory
    struct NARFF
    {
      typedef NARFF_Type Type; ///< IMPL Type
    };
    /// Feature Meta structure for use with PipelineFactory
    struct SHOT
    {
      typedef SHOT_Type Type; ///< IMPL Type
    };
    /// Feature Meta structure for use with PipelineFactory
    struct SI
    {
      typedef SI_Type Type; ///< IMPL Type
    };
#ifndef PCLREF_LEAN
    /// Feature Meta structure for use with PipelineFactory
    struct VFH
    {
      typedef VFH_Type Type; ///< IMPL Type
    };
    /// Feature Meta structure for use with PipelineFactory
    struct CVFH
    {
      typedef CVFH_Type Type; ///< IMPL Type
    };
    /// Feature Meta structure for use with PipelineFactory
    struct OURCVFH
    {
      typedef OURCVFH_Type Type; ///< IMPL Type
    };
    /// Feature Meta structure for use with PipelineFactory
    struct USC
    {
      typedef USC_Type Type; ///< IMPL Type
    };
    /// Feature Meta structure for use with PipelineFactory
    struct SC3D
    {
      typedef SC3D_Type Type; ///< IMPL Type
    };
    /// Feature Meta structure for use with PipelineFactory
    struct ESF
    {
      typedef ESF_Type Type; ///< IMPL Type
    };
#endif

    struct NoRemap {};///< Remap Meta structure for use with PipelineFactory
    struct Remap {};///< Remap Meta structure for use with PipelineFactory


    struct NoFeatureSpaceTransform {};///< feature space transformation Meta Structure for use with PipelineFactory
    struct FeatureSpacePCA         {};///< feature space transformation Meta Structure for use with PipelineFactory
    struct FeatureSpaceCDF         {};///< feature space transformation Meta Structure for use with PipelineFactory
    struct FeatureSpaceLRT         {};///< feature space transformation Meta Structure for use with PipelineFactory
    struct FeatureSpacePCAWrite    {};///< feature space transformation Meta Structure for use with PipelineFactory
    struct FeatureSpaceCDFWrite    {};///< feature space transformation Meta Structure for use with PipelineFactory
    struct FeatureSpaceLRTWrite    {};///< feature space transformation Meta Structure for use with PipelineFactory


    struct EstimateKeypointCorrespondences  {};///< Correspondence Estimation Meta Structure for use with PipelineFactory
    struct GTKeypointCorrespondences        {};///< Correspondence Estimation Meta Structure for use with PipelineFactory
    struct NoCorrespondences                {};///< Correspondence Estimation Meta Structure for use with PipelineFactory


    struct ICPAlign       {};///< Alignement Meta Structures for use with PipelineFactory
    struct NoRefine       {};///< Alignement Meta Structures for use with PipelineFactory
    struct NoAlign        {};///< Alignement Meta Structures for use with PipelineFactory
#ifndef PCLREF_LEAN
    struct NDTAlign       {};///< Alignement Meta Structures for use with PipelineFactory
#endif

    /// Pipeline Meta Structure for use with PipelineFactory. Encapsulates some useful typedefs.
    template<typename R, typename P, typename K, typename F,
             typename FSTransform = NoFeatureSpaceTransform,
             typename Correspondences = EstimateKeypointCorrespondences,
             typename Alignement = ICPAlign>
    struct PipelineMeta
    {
      typedef typename P::Type PointType; ///< TypeWrapper Type for Points
      typedef typename K::Type KeypointType; ///< TypeWrapper Type for Keypoints
      typedef typename F::Type FeatureType; ///< TypeWrapper Type for Features

      typedef ProcessingStep<PointType, KeypointType, FeatureType> StepType; ///< Type for a Pipeline Step
      typedef ProcessingContext<PointType, KeypointType, FeatureType> Context;  ///< Type of Context
      typedef Combiner<PointType, KeypointType, FeatureType> PipelineType;  ///< Type of Pipeline
    };

#define DEFINE_STEP_BUILDER(NAME, RType, PType, KType, FType, TType, CType, AType) \
  template<typename R, typename P, typename K, typename F, \
           typename T, typename C, typename A> \
  void NAME(PipelineMeta<RType, PType, KType, FType, TType, CType, AType>, \
   typename PipelineMeta<RType, PType, KType, FType, TType, CType, AType>::PipelineType* pipeline)


#define TYPEDEFS_STEP_BUILDER(RType, PType, KType, FType, TType, CType, AType) \
    typedef typename PipelineMeta<RType, PType, KType, FType, TType, CType, AType>::StepType::Ptr Ptr; \
    typedef typename PipelineMeta<RType, PType, KType, FType, TType, CType, AType>::PointType PointType; \
    typedef typename PipelineMeta<RType, PType, KType, FType, TType, CType, AType>::FeatureType FeatureType; \
    typedef typename PipelineMeta<RType, PType, KType, FType, TType, CType, AType>::KeypointType KeypointType;

#define DEFINE_PIPELINE_CREATE(NAME, RType, PType, KType, FType, TType, CType, AType) \
  template<typename R, typename P, typename K, typename F, \
           typename T, typename C, typename A> \
  bool NAME(PipelineMeta<RType, PType, KType, FType, TType, CType, AType>, \
  typename PipelineMeta<RType, PType, KType, FType, TType, CType, AType>::PipelineType*& pipeline)

#define TYPEDEFS_PIPELINE_CREATE(RType, PType, KType, FType, TType, CType, AType) \
    typedef PipelineMeta<RType, PType, KType, FType, TType, CType, AType> Meta;

    DEFINE_PIPELINE_CREATE(createCEPipeline, R, P, K, F, FeatureSpacePCAWrite, C, A)
    {
      pipeline = 0;
      return false;
    }
    DEFINE_PIPELINE_CREATE(createCEPipeline, R, P, K, F, FeatureSpaceCDFWrite, C, A)
    {
      pipeline = 0;
      return false;
    }
    DEFINE_PIPELINE_CREATE(createCEPipeline, R, P, K, F, T, C, A)
    {
      TYPEDEFS_PIPELINE_CREATE(R, P, K, F, T, C, A)
      pipeline = new typename Meta::PipelineType;
      return true;
    }

    DEFINE_PIPELINE_CREATE(createAccuPipeline, R, P, K, F, FeatureSpacePCAWrite, C, A)
    {
      TYPEDEFS_PIPELINE_CREATE(R, P, K, F, FeatureSpacePCAWrite, C, A)
      pipeline = new typename Meta::PipelineType;
      return true;
    }
    DEFINE_PIPELINE_CREATE(createAccuPipeline, R, P, K, F, FeatureSpaceCDFWrite, C, A)
    {
      TYPEDEFS_PIPELINE_CREATE(R, P, K, F, FeatureSpaceCDFWrite, C, A)
      pipeline = new typename Meta::PipelineType;
      return true;
    }
    DEFINE_PIPELINE_CREATE(createAccuPipeline, R, P, K, F, FeatureSpaceLRTWrite, C, A)
    {
      TYPEDEFS_PIPELINE_CREATE(R, P, K, F, FeatureSpaceLRTWrite, C, A)
      pipeline = new typename Meta::PipelineType;
      return true;
    }
    DEFINE_PIPELINE_CREATE(createAccuPipeline, R, P, K, F, T, C, ICPAlign)
    {
      TYPEDEFS_PIPELINE_CREATE(R, P, K, F, T, C, ICPAlign)
      pipeline = new typename Meta::PipelineType;
      return true;
    }
    DEFINE_PIPELINE_CREATE(createAccuPipeline, R, P, K, F, T, C, A)
    {
      pipeline = 0;
      return false;
    }

    DEFINE_STEP_BUILDER(addRemapStep, Remap, P, K, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(Remap, P, K, F, T, C, A)

      pipeline->addStep(Ptr(new RemapCloud<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addRemapStep, R, P, K, F, T, C, A)
    {
    }

    DEFINE_STEP_BUILDER(addPreprocessingStep, R, P, NARFK, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, NARFK, F, T, C, A)

      pipeline->addStep(Ptr(new ConvertToRangeImage<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addPreprocessingStep, R, P, K, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, T, C, A)

      pipeline->addStep(Ptr(new DownsampleCloud<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new EstimateNormals<PointType, KeypointType, FeatureType>()));
    }

    DEFINE_STEP_BUILDER(addPostprocessingStep, Remap, P, K, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(Remap, P, K, F, T, C, A)

      pipeline->addStep(Ptr(new RemoveRemap<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new CreateKeypointsFromIndices<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addPostprocessingStep, R, P, K, F, T, C, A)
    {
    }

    DEFINE_STEP_BUILDER(addKeypointStep, R, P, NoKeypoints, F, T, C, A)
    {
    }
    DEFINE_STEP_BUILDER(addKeypointStep, R, P, Harris, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, Harris, F, T, C, A)

      pipeline->addStep(Ptr(new Harris3DKeypoints<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new CreateKeypointIndices<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addKeypointStep, R, P, ISS, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, ISS, F, T, C, A)

      pipeline->addStep(Ptr(new ISSKeypoints<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new CreateKeypointIndices<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addKeypointStep, R, P, NARFK, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, NARFK, F, T, C, A)

      pipeline->addStep(Ptr(new NARFKeypoints<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new CreateKeypointsFromIndices<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addKeypointStep, R, P, UniformK, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, UniformK, F, T, C, A)

      pipeline->addStep(Ptr(new UniformKeypoints<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new CreateKeypointsFromIndices<PointType, KeypointType, FeatureType>()));
    }
#ifndef PCLREF_LEAN
    DEFINE_STEP_BUILDER(addKeypointStep, R, P, Region, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, Region, F, T, C, A)

      pipeline->addStep(Ptr(new GrowingRegions<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addKeypointStep, R, P, SIFT, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, SIFT, F, T, C, A)

      pipeline->addStep(Ptr(new SIFTKeypoints<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new CreateKeypointIndices<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addKeypointStep, R, P, RandomK, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, RandomK, F, T, C, A)

      pipeline->addStep(Ptr(new RandomKeypoints<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addKeypointStep, R, P, SS, F, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, SS, F, T, C, A)

      pipeline->addStep(Ptr(new SSKeypoints<PointType, FeatureType>()));
    }
#endif


    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, FPFH, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, FPFH, T, C, A)
      pipeline->addStep(Ptr(new FPFHFeatures<PointType, KeypointType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, NARFF, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, NARFF, T, C, A)
      pipeline->addStep(Ptr(new NARFFeatures<PointType, KeypointType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, SHOT, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, SHOT, T, C, A)
      pipeline->addStep(Ptr(new SHOTFeatures<PointType, KeypointType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, SI, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, SI, T, C, A)
      pipeline->addStep(Ptr(new SIFeatures<PointType, KeypointType>()));
    }
#ifndef PCLREF_LEAN
    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, CVFH, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, CVFH, T, C, A)
      pipeline->addStep(Ptr(new CVFHFeatures<PointType, KeypointType>()));
      pipeline->addStep(Ptr(new CreateKeypointsFromIndices<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, OURCVFH, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, OURCVFH, T, C, A)
      pipeline->addStep(Ptr(new EstimateNormals<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new OURCVFHFeatures<PointType, KeypointType>()));
      pipeline->addStep(Ptr(new CreateKeypointsFromIndices<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, VFH, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, VFH, T, C, A)
      pipeline->addStep(Ptr(new KeypointNeighborhood<PointType, KeypointType, FeatureType>()));
      //pipeline->addStep(Ptr(new KeypointClustering<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new VFHFeatures<PointType, KeypointType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, ESF, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, ESF, T, C, A)
      pipeline->addStep(Ptr(new KeypointNeighborhood<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new ESFFeatures<PointType, KeypointType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, SC3D, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, SC3D, T, C, A)
      pipeline->addStep(Ptr(new SC3DFeatures<PointType, KeypointType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureStep, R, P, K, USC, T, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, USC, T, C, A)
      pipeline->addStep(Ptr(new USCFeatures<PointType, KeypointType>()));
    }
#endif

    DEFINE_STEP_BUILDER(addFeatureSpaceStep, R, P, K, F, NoFeatureSpaceTransform, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, NoFeatureSpaceTransform, C, A)
    }
    DEFINE_STEP_BUILDER(addFeatureSpaceStep, R, P, K, F, FeatureSpacePCA, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, FeatureSpacePCA, C, A)
      pipeline->addStep(Ptr(new FeaturePCA<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureSpaceStep, R, P, K, F, FeatureSpaceLRT, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, FeatureSpaceLRT, C, A)
      pipeline->addStep(Ptr(new CDFTransform<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new LRTTransform<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureSpaceStep, R, P, K, F, FeatureSpaceCDF, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, FeatureSpaceCDF, C, A)
      pipeline->addStep(Ptr(new CDFTransform<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addFeatureSpaceStep, R, P, K, F, FeatureSpacePCAWrite, C, A)
    {
    }
    DEFINE_STEP_BUILDER(addFeatureSpaceStep, R, P, K, F, FeatureSpaceCDFWrite, C, A)
    {
    }
    DEFINE_STEP_BUILDER(addFeatureSpaceStep, R, P, K, F, FeatureSpaceLRTWrite, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, FeatureSpaceLRTWrite, C, A)
      pipeline->addStep(Ptr(new CDFTransform<PointType, KeypointType, FeatureType>()));
    }

    DEFINE_STEP_BUILDER(addCorrespondenceStep, R, P, K, F, T, GTKeypointCorrespondences, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, T, GTKeypointCorrespondences, A)
      pipeline->addStep(Ptr(new GTCorrespondences<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addCorrespondenceStep, R, P, K, F, T, EstimateKeypointCorrespondences, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, T, EstimateKeypointCorrespondences, A)
      pipeline->addStep(Ptr(new EstimateCorrespondences<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addCorrespondenceStep, R, P, K, F, T, NoCorrespondences, A)
    {
    }

    DEFINE_STEP_BUILDER(addAlignementStep, R, P, K, F, T, C, ICPAlign)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, T, C, ICPAlign)

      pipeline->addStep(Ptr(new Ransac<PointType, KeypointType, FeatureType>()));
      //pipeline->addStep(Ptr(new ICP<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addAccuStep, R, P, K, F, T, C, ICPAlign)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, T, C, ICPAlign)

      pipeline->addStep(Ptr(new MST<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new ICP<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addAlignementStep, R, P, K, F, T, C, NoRefine)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, T, C, NoRefine)

      pipeline->addStep(Ptr(new Ransac<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addAlignementStep, R, P, K, F, T, C, NoAlign)
    {
    }
#ifndef PCLREF_LEAN
    DEFINE_STEP_BUILDER(addAlignementStep, R, P, K, F, T, C, NDTAlign)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, T, C, NDTAlign)

      pipeline->addStep(Ptr(new Ransac<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new NDT<PointType, KeypointType, FeatureType>()));
    }
#endif

    DEFINE_STEP_BUILDER(addAccuStep, R, P, K, F, FeatureSpacePCAWrite, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, FeatureSpacePCAWrite, C, A)
      pipeline->addStep(Ptr(new FeaturePCA<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addAccuStep, R, P, K, F, FeatureSpaceCDFWrite, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, FeatureSpaceCDFWrite, C, A)
      pipeline->addStep(Ptr(new CDFTransform<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addAccuStep, R, P, K, F, FeatureSpaceLRTWrite, C, A)
    {
      TYPEDEFS_STEP_BUILDER(R, P, K, F, FeatureSpaceLRTWrite, C, A)
      pipeline->addStep(Ptr(new LRTTransform<PointType, KeypointType, FeatureType>()));
    }
    DEFINE_STEP_BUILDER(addAccuStep, R, P, K, F, T, C, A)
    {
    }
  }

  /**
   * \class PipelineFactory
   *
   * \brief This class can be used to build a pipeline using certain processing steps.
   *
   * The pipelines to be created are determined by the types used to instantiate PipelineFactory. For a full list of
   * these meta types consult the beginning of PipelineFactory.h.
   *
   * The following code taken from the example application in main.cc creates three pipelines.
   * A Keypoint-Feature pipeline with Harris Keypoint and SHOT features using XYZ points for the point cloud.
   * A Correspondence Estimation pipeline without Feature Space transformation (and because of ICPAlign also a
   * RANSAC alignment). And finally an Accumulation pipeline running ICP optimization on those cloud pairs
   * selected by MST. All are created in verbose mode, adding additional PipelineSteps for logging quality metrics.
   *
   * \code
   * PipelineFactory<NoRemap, XYZ, Harris, SHOT, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, ICPAlign> factory;
  * typename PipelineMeta::PipelineType* kfPipeline;
  * typename PipelineMeta::PipelineType* cePipeline;
  * typename PipelineMeta::PipelineType* accuPipeline;
  * factory.getKFPipeline(true, kfPipeline);
  * factory.getCEPipeline(true, cePipeline);
  * factory.getAccuPipeline(true, accuPipeline);
  * \endcode
  *
  * Each pipeline is actually a pointer to a Combiner object, though this is hidden
  * (since it is an implementation detail) inside the pclref::factoryMeta::PipelineMeta structure.
  */
  template<typename R, typename P, typename K, typename F,
           typename T = factoryMeta::NoFeatureSpaceTransform,
           typename C = factoryMeta::EstimateKeypointCorrespondences,
           typename A = factoryMeta::ICPAlign>
  class PipelineFactory
  {
    typedef typename factoryMeta::PipelineMeta<R, P, K, F, T, C, A>::PipelineType PipelineType;
    typedef typename factoryMeta::PipelineMeta<R, P, K, F, T, C, A>::StepType::Ptr Ptr;
    typedef typename factoryMeta::PipelineMeta<R, P, K, F, T, C, A>::PointType PointType;
    typedef typename factoryMeta::PipelineMeta<R, P, K, F, T, C, A>::FeatureType FeatureType;
    typedef typename factoryMeta::PipelineMeta<R, P, K, F, T, C, A>::KeypointType KeypointType;
    typedef typename factoryMeta::PipelineMeta<R, P, K, F, T, C, A>::Context ContextType;

  public:
    PipelineFactory()
    {
    }

    /// creates a Keypoint-Feature-Pipeline
    void getKFPipeline(bool verbose, PipelineType*& pipeline)
    {
      pipeline = new PipelineType();
      factoryMeta::PipelineMeta<R, P, K, F, T, C, A> m;

      pipeline->addStep(Ptr(new KFInfo<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new LoadGroundTruth<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new LoadCloud<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new TransformCloud<PointType, KeypointType, FeatureType>()));

      factoryMeta::addPreprocessingStep<R, P, K, F, T, C, A>(m, pipeline);
      factoryMeta::addRemapStep<R, P, K, F, T, C, A>(m, pipeline);

      if(verbose) pipeline->addStep(Ptr(new PointCloudStats<PointType, KeypointType, FeatureType>()));
      factoryMeta::addKeypointStep<R, P, K, F, T, C, A>(m, pipeline);
      factoryMeta::addFeatureStep<R, P, K, F, T, C, A>(m, pipeline);
      factoryMeta::addFeatureSpaceStep<R, P, K, F, T, C, A>(m, pipeline);
      factoryMeta::addPostprocessingStep<R, P, K, F, T, C, A>(m, pipeline);
      if(verbose) pipeline->addStep(Ptr(new Persistence<PointType, KeypointType, FeatureType>()));
      if(verbose) pipeline->addStep(Ptr(new UniquenessSelf<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new KeypointStatsSelf<PointType, KeypointType, FeatureType>()));
    }

    /// creates a Correspondence Estimation and Transformation Estimation Pipeline
    void getCEPipeline(bool verbose, PipelineType*& pipeline)
    {
      factoryMeta::PipelineMeta<R, P, K, F, T, C, A> m;
      if(!factoryMeta::createCEPipeline<R, P, K, F, T, C, A>(m, pipeline))
        return;

      pipeline->addStep(Ptr(new TInfo<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new CalculateOverlap<PointType, KeypointType, FeatureType>()));
      pipeline->addStep(Ptr(new Repeatability<PointType, KeypointType, FeatureType>()));
      if(verbose)
      {
        pipeline->addStep(Ptr(new Uniqueness<PointType, KeypointType, FeatureType>()));
        pipeline->addStep(Ptr(new SampleMU<PointType, KeypointType, FeatureType>()));
      }
      factoryMeta::addCorrespondenceStep<R, P, K, F, T, C, A>(m, pipeline);
      factoryMeta::addAlignementStep<R, P, K, F, T, C, A>(m, pipeline);
      pipeline->addStep(Ptr(new CorrespondenceInlierRate<PointType, KeypointType, FeatureType>()));
    }

    /// creates an Accumulation Pipeline (used for PCA, LRT training, MST calculation and/or ICP optimization)
    void getAccuPipeline(bool verbose, PipelineType*& pipeline)
    {
      factoryMeta::PipelineMeta<R, P, K, F, T, C, A> m;
      if(!factoryMeta::createAccuPipeline<R, P, K, F, T, C, A>(m, pipeline))
        return;

      factoryMeta::addAccuStep<R, P, K, F, T, C, A>(m, pipeline);
    }
  };

}

#endif
