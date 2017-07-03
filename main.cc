#include <PclRefLIB.h>
#include <DataMerge.h>
#include <DataTable.h>
#include <FileTools.h>

#include <ParameterTemplate.h>
#include <PipelineFactory.h>

#include <boost/assign/std/vector.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace boost::assign;
using namespace pclref;
using namespace pclref::factoryMeta;

/// yes, it finds files
void findFiles(ParameterContext::Ptr params, const std::string& root_folder, std::vector<std::string>& files)
{
  std::string ext = params->template get<std::string>("find_files_extension");
  if(params->template get<bool>("find_files_recursive"))
  {
    FileTools::findFilesRecursive(root_folder, ext, files);
  }
  else
  {
    FileTools::findFiles(root_folder, ext, files);
  }
}

/// parses a config-like file list
void parseFileList(const std::string& filename, std::vector<std::string>& files)
{
  std::ifstream file(filename.c_str());
  std::string line;
  while( std::getline(file, line) )
  {
    if(line[0] == '#')
      continue;
    files.push_back(line);
  }
  file.close();
}

/// holds some info about a specific pipeline run
struct PipelineRun
{
  std::string cloud_file; ///< either a pcd or csv file
  std::string gt_file; ///< a file containing ground truth transformation in a csv-like format
  int gt_line; ///< line (row0) in the ground truth file of this cloud
  int gt_col; ///< column (col0) in the ground truth file of this cloud
  bool gt_inline; ///< inline or rectangular ground truth parsing
  bool from_matrix; ///< ground truth is present as homogenous matrix (translation+euler else)
  bool cdf_stats_available; ///< there is a cdf stats file
  typename DataAccessor::Ptr cdf_stats; ///< there pre-parsed cdf stats
  bool pca_stats_available; ///< there is a pca stats file
  typename DataAccessor::Ptr pca_stats; ///< there pre-parsed pca stats
  bool lrt_stats_available; ///< there is a lrt stats file
  typename DataAccessor::Ptr lrt_stats; ///< there pre-parsed lrt stats
};

/// parses a session file and stats
void parseSessionFile(const std::string& filename, const std::string& cdf_stats_file,
                      const std::string& pca_stats_file, const std::string& lrt_stats_file,
                      std::vector<PipelineRun>& runs)
{
  typename DataTable::Ptr table(new DataTable);
  table->fromCSV(filename, true);

  bool cdf_stats_available = cdf_stats_file != "";
  typename DataAccessor::Ptr cdf_stats(new DataTable);
  if(cdf_stats_available)
  {
    ((DataTable*)cdf_stats.get())->fromCSV(cdf_stats_file, true);
  }

  bool pca_stats_available = pca_stats_file != "";
  typename DataAccessor::Ptr pca_stats(new DataTable);
  if(pca_stats_available)
  {
    ((DataTable*)pca_stats.get())->fromCSV(pca_stats_file, true);
  }

  bool lrt_stats_available = lrt_stats_file != "";
  typename DataAccessor::Ptr lrt_stats(new DataTable);
  if(lrt_stats_available)
  {
    ((DataTable*)lrt_stats.get())->fromCSV(lrt_stats_file, true);
  }

  for(int i = 0; i < table->getRowCount(); ++i)
  {
    PipelineRun run;
    run.cloud_file = table->atEnum(i, 0);
    run.gt_file = table->atEnum(i, 1);
    run.gt_line = table->at(i, 2);
    run.gt_col = table->at(i, 3);
    run.gt_inline = table->at(i, 4) > 0;
    run.from_matrix = table->at(i, 5) > 0;
    run.cdf_stats_available = cdf_stats_available;
    run.cdf_stats = cdf_stats_available ? cdf_stats : typename DataAccessor::Ptr();
    run.pca_stats_available = pca_stats_available;
    run.pca_stats = pca_stats_available ? pca_stats : typename DataAccessor::Ptr();
    run.lrt_stats_available = lrt_stats_available;
    run.lrt_stats = lrt_stats_available ? lrt_stats : typename DataAccessor::Ptr();
    runs.push_back(run);
  }
}

/// creates pipeline contexts from a set of pipeline runs
template<typename R, typename P, typename K, typename F,
         typename T, typename C, typename A>
void registerRuns(PipelineMeta<R, P, K, F, T, C, A> meta,
                  typename PipelineMeta<R, P, K, F, T, C, A>::Context::Ptr contexts,
                  ParameterContext::Ptr params, const std::vector<PipelineRun>& runs)
{
  typedef PipelineMeta<R, P, K, F, T, C, A> PipelineMeta;
  typedef typename PipelineMeta::StepType::Ptr Ptr;
  typedef typename PipelineMeta::PointType PointType;
  typedef typename PipelineMeta::FeatureType FeatureType;
  typedef typename PipelineMeta::Context ContextType;

  char seperator = params->template get<char>("file_parse_seperator");
  int split = params->template get<int>("file_parse_split");
  bool has_header = params->template get<bool>("file_has_header");

  if(params->get<bool>("parse_csv"))
  {
    // parse all csv files, use splitter on them and put into context
    for(int i = 0, j = 0; i < runs.size(); ++i)
    {
      PipelineRun run = runs[i];

      DataAccessor::Ptr cloud_table(new DataTable());
      DataAccessor::Ptr gt_table(new DataTable());

      ((DataTable*)cloud_table.get())->fromCSV(run.cloud_file, has_header, seperator);
      ((DataTable*)gt_table.get())->fromCSV(run.gt_file, has_header, seperator);

      if(split >= 0)
      {
        std::vector<typename DataAccessor::Ptr> accessors;
        cloud_table->splitByValue((size_t)split, accessors);
        BOOST_FOREACH(DataAccessor::Ptr acc, accessors)
        {
          typename ContextType::Ptr context(new ContextType);
          context->set("load_cloud_structure", acc);
          context->set("id", (identifier)(j+1));
          context->set("gt_structure", gt_table);
          context->set("gt_col", run.gt_col);
          context->set("gt_line", run.gt_line);
          context->set("gt_inline", run.gt_inline);
          context->set("gt_from_matrix", run.from_matrix);
          if(run.cdf_stats_available) context->set("cdf_stats", run.cdf_stats);
          if(run.pca_stats_available) context->set("pca_stats", run.pca_stats);
          if(run.lrt_stats_available) context->set("lrt_stats", run.lrt_stats);
          contexts->add("context_" + precision_cast(j++, 1), context);
        }
      }
      else
      {
        typename ContextType::Ptr context(new ContextType);
        context->set("load_cloud_structure", cloud_table);
        context->set("id", (identifier)(j+1));
        context->set("gt_structure", gt_table);
        context->set("gt_col", run.gt_col);
        context->set("gt_line", run.gt_line);
        context->set("gt_inline", run.gt_inline);
        context->set("gt_from_matrix", run.from_matrix);
        if(run.cdf_stats_available) context->set("cdf_stats", run.cdf_stats);
        if(run.pca_stats_available) context->set("pca_stats", run.pca_stats);
        if(run.lrt_stats_available) context->set("lrt_stats", run.lrt_stats);
        contexts->add("context_" + precision_cast(j++, 1), context);
      }
    }
  }
  else
  {
    for(int i = 0; i < runs.size(); ++i)
    {
      PipelineRun run = runs[i];
      DataAccessor::Ptr gt_table(new DataTable());
      ((DataTable*)gt_table.get())->fromCSV(run.gt_file, has_header, seperator);

      typename ContextType::Ptr context(new ContextType);
      context->set("load_cloud_structure", run.cloud_file);
      context->set("id", (identifier)(i+1));
      context->set("gt_structure", gt_table);
      context->set("gt_col", run.gt_col);
      context->set("gt_line", run.gt_line);
      context->set("gt_inline", run.gt_inline);
      context->set("gt_from_matrix", run.from_matrix);
      if(run.cdf_stats_available) context->set("cdf_stats", run.cdf_stats);
      if(run.pca_stats_available) context->set("pca_stats", run.pca_stats);
      if(run.lrt_stats_available) context->set("lrt_stats", run.pca_stats);
      contexts->add("context_" + precision_cast(i, 1), context);
    }
  }

}

/// a flatten inspired function to make one dataacessor out of several
typename DataAccessor::Ptr flatten(const std::vector<typename DataAccessor::Ptr> data)
{
  int idx = 0;
  std::vector<typename DataAccessor::Ptr> enhanced_data;
  for(int i = 0; i < data.size(); ++i)
  {
    std::vector<DataTable::columnType> types;
    types.push_back(DataTable::FLOAT);
    types.push_back(DataTable::ENUM);
    std::vector<std::string> names;
    names.push_back("id");
    names.push_back("log_name");
    typename DataTable::Ptr leftColumn(new DataTable(types, names));
    for(int j = 0; j < data[i]->getRowCount(); ++j)
    {
      leftColumn->set(j, 0, idx++);
      leftColumn->set(j, 1, data[i]->getName());
    }

    typename DataMerge::Ptr add(new DataMerge(false));
    add->addAccessor(leftColumn);
    add->addAccessor(data[i]);
    enhanced_data.push_back(add);
  }

  typename DataMerge::Ptr result = typename DataMerge::Ptr(new DataMerge(true));
  for(int i = 0; i < enhanced_data.size(); ++i)
  {
    result->addAccessor(enhanced_data[i]);
  }
  return result;
}

/// This function runs the pipelines specified
template<typename R, typename P, typename K, typename F,
         typename T, typename C, typename A>
void runPipeline(PipelineMeta<R, P, K, F, T, C, A> meta,
                 const std::string& session_file,
                 const std::vector<ParameterContext::Ptr>& pcontexts,
                 bool verbose)
{
  typedef PipelineMeta<R, P, K, F, T, C, A> PipelineMeta;
  typedef typename PipelineMeta::StepType::Ptr Ptr;
  typedef typename PipelineMeta::PointType PointType;
  typedef typename PipelineMeta::FeatureType FeatureType;
  typedef typename PipelineMeta::KeypointType KeypointType;
  typedef typename PipelineMeta::Context ContextType;

  PipelineFactory<R, P, K, F, T, C, A> factory;
  typename PipelineMeta::PipelineType* kfPipeline;
  typename PipelineMeta::PipelineType* cePipeline;
  typename PipelineMeta::PipelineType* accuPipeline;
  factory.getKFPipeline(verbose, kfPipeline);
  factory.getCEPipeline(verbose, cePipeline);
  factory.getAccuPipeline(verbose, accuPipeline);


  // find input files. if csv-like files (instead of proper pcd) are given, preparse them.
  BOOST_FOREACH(ParameterContext::Ptr params, pcontexts)
  {
    std::string output_folder = params->template get<std::string>("output_folder");

    std::vector<PipelineRun> runs;
    parseSessionFile(session_file,
                     params->get<std::string>("cdf_stats_file"),
                     params->get<std::string>("pca_stats_file"),
                     params->get<std::string>("lrt_stats_file"),
                     runs);

    // create contexts for all pipelines
    typename ContextType::Ptr kfContexts(new ContextType);
    typename ContextType::Ptr ceContexts(new ContextType);
    typename ContextType::Ptr accuContexts(new ContextType);

    registerRuns(meta, kfContexts, params, runs);

    if(kfPipeline)
    {
      // run keypoint/feature pipeline
      {
        kfPipeline->setParameterContext(params);
        bool success = kfPipeline->compute(0, kfContexts);
        if(!success)
          continue;
      }

      // write feature cloud pipeline log to csv file
      if(params->get<bool>("write_kfpipeline_info"))
      {
        typename DataAccessor::Ptr additional_log = flatten(kfPipeline->getAdditionalLogs());

        if(!output_folder.empty()) FileTools::createFolder(output_folder);
        FileTools::createFolder(output_folder + kfPipeline->getName());
        FileTools::createFolder(output_folder + kfPipeline->getName() + "/" + params->getName());
        FileTools::createFolder(output_folder + kfPipeline->getName() + "/" + params->getName() + "/" +
                                precision_cast(params->template get<int>("seed")));

        std::string file_base = output_folder + kfPipeline->getName() + "/" + params->getName()+ "/" +
            precision_cast(params->template get<int>("seed")) + "/kf";

        std::string filename = file_base + "log.csv";
        kfPipeline->getLogger()->toCSV(filename);
        filename = file_base + "additional_log.csv";
        additional_log->toCSV(filename, 10);
      }
    }

    if(cePipeline)
    {
      // fill ce pipeline context
      typename ContextType::iterator iterI = kfContexts->begin();
      typename ContextType::iterator iend = kfContexts->end();
      for(int k = 0, r = 0; iterI != iend; ++iterI, ++r)
      {
        typename ContextType::iterator iend2 = kfContexts->end();
        typename ContextType::iterator iterJ;
        if(params->get<int>("diagonal_pipeline") > 0)
        {
          iterJ = iterI - std::min(r, params->get<int>("diagonal_pipeline"));
          iend2 = iterI + std::min((int)kfContexts->size()-r, params->get<int>("diagonal_pipeline")+1);
        }
        else
          iterJ = kfContexts->begin();

        for(; iterJ != iend2; ++iterJ)
        {
          if(iterJ == iterI)
            continue;

          typename ContextType::Ptr context(new ContextType);

          typename ContextType::Ptr sourceContext = boost::any_cast<typename ContextType::Ptr>(*iterJ);
          typename ContextType::Ptr targetContext = boost::any_cast<typename ContextType::Ptr>(*iterI);

          context->add("source_id", sourceContext->template get<identifier>("id"));
          context->add("target_id", targetContext->template get<identifier>("id"));

          context->add("cloud_source", sourceContext->template get<typename ContextType::PointCloud::Ptr>("cloud"));
          context->add("cloud_target", targetContext->template get<typename ContextType::PointCloud::Ptr>("cloud"));
          context->add("ground_truth_source", sourceContext->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth"));
          context->add("ground_truth_target", targetContext->template get<boost::shared_ptr<typename ContextType::Transformation> >("ground_truth"));

          context->add("keypoints_source", sourceContext->template get<typename ContextType::KeypointCloud::Ptr>("keypoints"));
          context->add("keypoints_target", targetContext->template get<typename ContextType::KeypointCloud::Ptr>("keypoints"));
          context->add("keypoint_indices_source", sourceContext->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices"));
          context->add("keypoint_indices_target", targetContext->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices"));

          context->add("features_source", sourceContext->template get<typename ContextType::FeatureCloud::Ptr>("features"));
          context->add("features_target", targetContext->template get<typename ContextType::FeatureCloud::Ptr>("features"));

          ceContexts->add("context_" + precision_cast(k++, 1), context);
        }
      }

      // run correspondence estimation pipeline
      {
        cePipeline->setParameterContext(params);

        bool success = cePipeline->compute(0, ceContexts);
        if(!success)
          continue;
      }

      // write correspondence estimation log to csv file
      if(params->get<bool>("write_cepipeline_info"))
      {
        typename DataAccessor::Ptr additional_log = flatten(cePipeline->getAdditionalLogs());

        if(!output_folder.empty()) FileTools::createFolder(output_folder);
        FileTools::createFolder(output_folder + kfPipeline->getName());
        FileTools::createFolder(output_folder + kfPipeline->getName() + "/" + params->getName());
        FileTools::createFolder(output_folder + kfPipeline->getName() + "/" + params->getName() + "/" +
                                precision_cast(params->template get<int>("seed")));

        std::string file_base = output_folder + kfPipeline->getName() + "/" + params->getName()+ "/" +
            precision_cast(params->template get<int>("seed")) + "/ce";

        std::string filename = file_base + "log.csv";
        cePipeline->getLogger()->toCSV(filename);
        filename = file_base + "additional_log.csv";
        additional_log->toCSV(filename, 10);
      }
    }

    if(accuPipeline)
    {
      // just put kf context inside accuContexts
      // so the accu pipeline just gets a single execution
      // and the accu pipeline steps get a context of contexts
      {
        typename ContextType::Ptr accu(new ContextType);
        accu->add("kfContexts", kfContexts);
        accu->add("ceContexts", ceContexts);
        accu->add("accu", true);
        accuContexts->add("context", accu);
      }

      // run accumulation pipeline
      {
        accuPipeline->setParameterContext(params);

        bool success = accuPipeline->compute(0, accuContexts);
        if(!success)
          continue;
      }

      // write accu log to csv file
      if(params->get<bool>("write_accupipeline_info"))
      {
        typename DataAccessor::Ptr additional_log = flatten(accuPipeline->getAdditionalLogs());

        if(!output_folder.empty()) FileTools::createFolder(output_folder);
        FileTools::createFolder(output_folder + kfPipeline->getName());
        FileTools::createFolder(output_folder + kfPipeline->getName() + "/" + params->getName());
        FileTools::createFolder(output_folder + kfPipeline->getName() + "/" + params->getName() + "/" +
                                precision_cast(params->template get<int>("seed")));

        std::string file_base = output_folder + kfPipeline->getName() + "/" + params->getName()+ "/" +
            precision_cast(params->template get<int>("seed")) + "/accu";

        std::string filename = file_base + "log.csv";
        accuPipeline->getLogger()->toCSV(filename);
        filename = file_base + "additional_log.csv";
        additional_log->toCSV(filename, 10);
      }
    }

    if(params->get<bool>("visualize"))
    {
      Visualizer<PointType, KeypointType, FeatureType> vis;      
      typename ContextType::Ptr accu = boost::any_cast<typename ContextType::Ptr>(*accuContexts->begin());
      if(accu->exists("mstContext"))
          vis.setFullContext(accu->template get<typename ContextType::Ptr>("mstContext"));
      vis.startVisualization(ceContexts);
    }

    // reset pipelines
    if(cePipeline) cePipeline->init();
    if(kfPipeline) kfPipeline->init();
    if(accuPipeline) accuPipeline->init();
  }


  // delete pipelines
  if(cePipeline) delete cePipeline;
  if(kfPipeline) delete kfPipeline;
  if(accuPipeline) delete accuPipeline;
}

// Pipeline Runners. These are only used to create a Pipeline Meta structure and call runPipeline.
// Refactoring them is a real ___. (Currently these functions are put into a map and called by a
// string given to the app. That's slightly better than a long list of if/else statements.)
#define RUNNER_PARAMS const std::string& session_file, const std::vector<ParameterContext::Ptr>& pcontexts, bool verbose
#define RUNNER_COMMAND runPipeline(meta, session_file, pcontexts, verbose)
// 1. Harris Pipelines
// 1.1 Different Feature Descriptors
void runHarrisFPFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runHarrisSHOT(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, SHOT, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runHarrisSI(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, SI, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
#ifndef PCLREF_LEAN
void runHarrisESF(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, ESF, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runHarrisSC3D(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, SC3D, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runHarrisUSC(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, USC, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runHarrisVFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, ESF, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
#endif
// 1.2 Remapping
void runRemappedHarrisFPFH(RUNNER_PARAMS)
{
  PipelineMeta<Remap, XYZ, Harris, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
// 1.3 Feature Space Transformations
void runHarrisFPFHPCA(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, FeatureSpacePCA, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runHarrisFPFHPCAWrite(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, FeatureSpacePCAWrite, NoCorrespondences, NoAlign> meta;
  RUNNER_COMMAND;
}
void runHarrisSHOTPCA(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, SHOT, FeatureSpacePCA, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runHarrisSHOTPCAWrite(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, SHOT, FeatureSpacePCAWrite, NoCorrespondences, NoAlign> meta;
  RUNNER_COMMAND;
}
void runHarrisFPFHLRT(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, FeatureSpaceLRT, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runHarrisFPFHLRTWrite(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, FeatureSpaceLRTWrite, NoCorrespondences, NoAlign> meta;
  RUNNER_COMMAND;
}
void runHarrisFPFHCDF(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, FeatureSpaceCDF, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runHarrisFPFHCDFWrite(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, FeatureSpaceCDFWrite, NoCorrespondences, NoAlign> meta;
  RUNNER_COMMAND;
}
// 1.3.1 ISS Feature Space Transformations
void runISSFPFHPCA(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, ISS, FPFH, FeatureSpacePCA, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runISSFPFHPCAWrite(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, ISS, FPFH, FeatureSpacePCAWrite, NoCorrespondences, NoAlign> meta;
  RUNNER_COMMAND;
}
void runISSFPFHLRT(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, ISS, FPFH, FeatureSpaceLRT, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runISSFPFHLRTWrite(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, ISS, FPFH, FeatureSpaceLRTWrite, NoCorrespondences, NoAlign> meta;
  RUNNER_COMMAND;
}
void runISSFPFHCDF(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, ISS, FPFH, FeatureSpaceCDF, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runISSFPFHCDFWrite(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, ISS, FPFH, FeatureSpaceCDFWrite, NoCorrespondences, NoAlign> meta;
  RUNNER_COMMAND;
}
// 1.4 Ground Truth Correspondences
void runHarrisFPFHGT(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, NoFeatureSpaceTransform, GTKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
// 1.5 Alignment Optimization
void runHarrisFPFHICP(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, ICPAlign> meta;
  RUNNER_COMMAND;
}
void runHarrisSHOTICP(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, SHOT, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, ICPAlign> meta;
  RUNNER_COMMAND;
}
#ifndef PCLREF_LEAN
void runHarrisFPFHNDT(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Harris, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NDTAlign> meta;
  RUNNER_COMMAND;
}
#endif

// 2. ISS Keypoints
void runISSFPFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, ISS, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runISSSHOT(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, ISS, SHOT, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runRemappedISSFPFH(RUNNER_PARAMS)
{
  PipelineMeta<Remap, XYZ, ISS, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}

#ifndef PCLREF_LEAN
// 3. SS Keypoints
void runSSFPFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, SS, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}

// 4. SIFT Keypoints
void runSIFTFPFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, SIFT, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runSIFTVFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, SIFT, VFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
#endif

// 5. Uniform and Random Keypoints
void runUniformFPFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, UniformK, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
#ifndef PCLREF_LEAN
void runRandomFPFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, RandomK, FPFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}

// 6. Clustering
void runRegionVFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, Region, VFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runCVFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, NoKeypoints, CVFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
void runOURCVFH(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, NoKeypoints, OURCVFH, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}
#endif

// 7. NARF
void runNARF(RUNNER_PARAMS)
{
  PipelineMeta<NoRemap, XYZ, NARFK, NARFF, NoFeatureSpaceTransform, EstimateKeypointCorrespondences, NoRefine> meta;
  RUNNER_COMMAND;
}

int main (int argc, char** argv)
{
  std::map<std::string, boost::function<void (RUNNER_PARAMS)> > pipeline_starters;
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH", runHarrisFPFH));
  pipeline_starters.insert(std::make_pair("HARRIS_SHOT", runHarrisSHOT));
  pipeline_starters.insert(std::make_pair("HARRIS_SI", runHarrisSI));
  pipeline_starters.insert(std::make_pair("REMAPPED_HARRIS_FPFH", runRemappedHarrisFPFH));
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH_PCA", runHarrisFPFHPCA));
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH_PCA_WRITE", runHarrisFPFHPCAWrite));
  pipeline_starters.insert(std::make_pair("HARRIS_SHOT_PCA", runHarrisSHOTPCA));
  pipeline_starters.insert(std::make_pair("HARRIS_SHOT_PCA_WRITE", runHarrisSHOTPCAWrite));
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH_CDF", runHarrisFPFHCDF));
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH_CDF_WRITE", runHarrisFPFHCDFWrite));
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH_LRT", runHarrisFPFHLRT));
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH_LRT_WRITE", runHarrisFPFHLRTWrite));
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH_GT", runHarrisFPFHGT));
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH_ICP", runHarrisFPFHICP));
  pipeline_starters.insert(std::make_pair("HARRIS_SHOT_ICP", runHarrisSHOTICP));

  pipeline_starters.insert(std::make_pair("ISS_FPFH", runISSFPFH));
  pipeline_starters.insert(std::make_pair("ISS_SHOT", runISSSHOT));
  pipeline_starters.insert(std::make_pair("REMAPPED_ISS_FPFH", runRemappedISSFPFH));
  pipeline_starters.insert(std::make_pair("ISS_FPFH_PCA_WRITE", runISSFPFHPCAWrite));
  pipeline_starters.insert(std::make_pair("ISS_FPFH_PCA", runISSFPFHPCA));
  pipeline_starters.insert(std::make_pair("ISS_FPFH_CDF_WRITE", runISSFPFHCDFWrite));
  pipeline_starters.insert(std::make_pair("ISS_FPFH_CDF", runISSFPFHCDF));
  pipeline_starters.insert(std::make_pair("ISS_FPFH_LRT_WRITE", runISSFPFHLRTWrite));
  pipeline_starters.insert(std::make_pair("ISS_FPFH_LRT", runISSFPFHLRT));
  pipeline_starters.insert(std::make_pair("UNIFORM_FPFH", runUniformFPFH));

#ifndef PCLREF_LEAN
  pipeline_starters.insert(std::make_pair("HARRIS_ESF", runHarrisESF));
  pipeline_starters.insert(std::make_pair("HARRIS_SC3D", runHarrisSC3D));
  pipeline_starters.insert(std::make_pair("HARRIS_USC", runHarrisUSC));
  pipeline_starters.insert(std::make_pair("HARRIS_VFH", runHarrisVFH));
  pipeline_starters.insert(std::make_pair("HARRIS_FPFH_NDT", runHarrisFPFHNDT));
  pipeline_starters.insert(std::make_pair("SS_FPFH", runSSFPFH));
  pipeline_starters.insert(std::make_pair("SIFT_FPFH", runSIFTFPFH));
  pipeline_starters.insert(std::make_pair("SIFT_VFH", runSIFTVFH));
  pipeline_starters.insert(std::make_pair("RANDOM_FPFH", runRandomFPFH));
  pipeline_starters.insert(std::make_pair("REGION_VFH", runRegionVFH));
  pipeline_starters.insert(std::make_pair("CVFH", runCVFH));
  pipeline_starters.insert(std::make_pair("OURCVFH", runOURCVFH));
  pipeline_starters.insert(std::make_pair("NARF", runNARF));
#endif

  pipeline_starters.insert(std::make_pair("fallback", runHarrisFPFH));

  // file parsing
  bool parse = false;
  char seperator = ' ';    // seperator for csv-like parsing (instead of standard pcl file loading)
  int split = -1;          // split for values by this column (if multiple pointclouds in one csv-like file)
  bool has_header = false; // has csv file a header? (first line with names / currently i also support name:type)
  int repeats = 1;
  bool verbose = false;
  int threads = 1;
  int diagonal = 0;

  // Declare a group of options that will be
  // allowed only on command line
  po::options_description generic("Generic options");
  generic.add_options()
  ("help", "produce help message");

  // Config options
  po::options_description config("Configuration");
  config.add_options()
  ("pipeline,P", po::value<std::string >(), "choose pipeline to use")
  ("parameter-file,p", po::value<std::string>(), "path to a parameter file")
  ("session-file,S", po::value<std::string>(), "path to the session file")
  ("cdf-stats-file", po::value<std::string>(), "a cdf file for inverse cdf transform")
  ("pca-stats-file", po::value<std::string>(), "a pca file for pca transform and whitening")
  ("lrt-stats-file", po::value<std::string>(), "a lrt file for lrt transform")
  ("correspondence-predictor,C", po::value<std::string>(), "xml file of a opencv decision tree")
  ("repeat,r",po::value<int>(&repeats)->default_value(1), "how many times the alignement should be run")
  ("diagonal",po::value<int>(&diagonal)->default_value(0), "if greater than zero only the nth next point clouds are aligned")
  ("output-folder,O", po::value<std::string>(), "base folder for outputting")
  ("csv-parse,c", "parse files as csv-like files")
  ("seperator,s", po::value<char>(&seperator)->default_value(' '), "seperator for csv-like files")
  ("split", po::value<int>(&split)->default_value(-1), "split csv files by values from this column (if multiple clouds in one csv-file)")
  ("threads,t", po::value<int>(&threads)->default_value(1), "split csv files by values from this column (if multiple clouds in one csv-file)")
  ("verbose,v", "verbose mode")
  ("has-header", "has csv file a header?");

  po::options_description cmdline_options;
  cmdline_options.add(generic).add(config);

  po::options_description config_file_options;
  config_file_options.add(config);

  po::options_description visible("Allowed options");
  visible.add(generic).add(config);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv)
            .options(cmdline_options)
            .run(),
            vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << visible << std::endl;
    return 1;
  }

  std::string parameter_file = "";
  if(vm.count("parameter-file"))
  {
    parameter_file = vm["parameter-file"].as<std::string>();
  }
  std::string session_file = "";
  if(vm.count("session-file"))
  {
    session_file = vm["session-file"].as<std::string>();
  }
  std::string cdf_stats_file = "";
  if(vm.count("cdf-stats-file"))
  {
    cdf_stats_file = vm["cdf-stats-file"].as<std::string>();
  }
  std::string pca_stats_file = "";
  if(vm.count("pca-stats-file"))
  {
    pca_stats_file = vm["pca-stats-file"].as<std::string>();
  }
  std::string lrt_stats_file = "";
  if(vm.count("lrt-stats-file"))
  {
    lrt_stats_file = vm["lrt-stats-file"].as<std::string>();
  }
  std::string predictor_file = "";
  if(vm.count("correspondence-predictor"))
  {
    predictor_file = vm["correspondence-predictor"].as<std::string>();
  }
  std::string output_folder = "";
  if(vm.count("output-folder"))
  {
    output_folder = vm["output-folder"].as<std::string>() + "/";
  }

  parse = vm.count("csv-parse") > 0;
  has_header = vm.count("has-header") > 0;
  verbose = vm.count("verbose") > 0;

  ParameterTemplate::Ptr params(new ParameterTemplate);
  params->setName("std");
  params->set("diagonal_pipeline", diagonal);
  params->set("write_kfpipeline_info", true);
  params->set("write_cepipeline_info", true);
  params->set("write_accupipeline_info", true);
  params->set("visualize", false);
  params->set("threads", threads);

  // file parsing
  params->set("parse_csv", parse);
  params->set("file_parse_seperator", seperator);
  params->set("file_parse_split", split);
  params->set("file_has_header", has_header);

  // feature statistics
  params->set("cdf_stats_file", cdf_stats_file);
  params->set("pca_stats_file", pca_stats_file);
  params->set("lrt_stats_file", lrt_stats_file);

  params->set("predictor_file", predictor_file);
  params->set("output_folder", output_folder);
  params->set("verbose", verbose);

  if(!parameter_file.empty())
    params->load(parameter_file);

  for(int i = 0; i < repeats; ++i)
  {
    std::string name_base = params->getName();
    std::vector<ParameterContext::Ptr> pcontexts;
    params->generate(pcontexts);

    for(size_t idx = 0; idx < pcontexts.size(); ++idx)
      pcontexts[idx]->set("seed", i);

    params->setName(name_base);

    std::string pipe = vm.count("pipeline") > 0 ? vm["pipeline"].as<std::string>() : "fallback";
    if(pipeline_starters.find(pipe) != pipeline_starters.end())
    {
      std::cout << "running: " << pipe << " pipeline" << std::endl;
      pipeline_starters[pipe](session_file, pcontexts, verbose);
    }
    else
    {
      std::cout << pipe << " not recognized!" << std::endl;
      std::cout << "running: " << pipe << " pipeline" << std::endl;
      pipeline_starters["fallback"](session_file, pcontexts, verbose);
    }
  }
  return (0);
}
