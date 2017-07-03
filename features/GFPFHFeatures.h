#if 0
/*
 * vfh_features.h
 *
 *  Created on: September 3, 2014
 *      Author: max
 */

#ifndef GFPFH_FEATURES_H_
#define GFPFH_FEATURES_H_

#include <ElysiumLIB.h>
#include <DataTable.h>
#include <ProcessingStep.h>

#include <pcl/features/gfpfh.h>

namespace elysium
{
  template<typename PointType, typename KeypointType>
  class GFPFHFeatures : public ProcessingStep<PointType, KeypointType, pcl::GFPFHSignature16>
  {
  public:
    typedef ProcessingStep<PointType, KeypointType, pcl::GFPFHSignature16> BaseType;
    using typename BaseType::Ptr;

    typedef ProcessingContext<PointType, KeypointType, pcl::GFPFHSignature16> ContextType;

    GFPFHFeatures()
    {
      init();
    }

    void init()
    {
      BaseType::init();

      std::vector<DataTable::columnType> types;
      types.push_back(DataTable::FLOAT);
      types.push_back(DataTable::FLOAT);
      std::vector<std::string> names;
      names.push_back("gfpfh_time");
      names.push_back("gfpfh_size");
      this->m_logger = boost::shared_ptr<DataTable>(new DataTable(types, names));
    }

    bool compute()
    {
      int row = this->m_logger->getRowCount();
      this->m_logger->addRow();
      const clock_t t0 = clock();

      typename ContextType::PointCloud::Ptr cloud = this->m_context->template get<typename ContextType::PointCloud::Ptr>("cloud");
      typename ContextType::IndexCloud::Ptr labels = this->m_context->template get<typename ContextType::IndexCloud::Ptr>("labels");
      typename ContextType::IndexCloud::Ptr keypoint_indices = this->m_context->template get<typename ContextType::IndexCloud::Ptr>("keypoint_indices");
      typename ContextType::FeatureCloud::Ptr features(new typename ContextType::FeatureCloud);

      boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
      indices->reserve(keypoint_indices->size());
      BOOST_FOREACH(int index, *keypoint_indices)
      {
        indices->push_back(index);
      }

      pcl::GFPFHEstimation<PointType, int, pcl::GFPFHSignature16> gfpfh;
      gfpfh.setIndices(indices);
      gfpfh.setInputCloud(cloud);
      float model_resolution = this->m_params->template get<double>("model_resolution", 0.05);
      gfpfh.setRadiusSearch(this->m_params->template get<double>("gfpfh_radius_search", 6*model_resolution));
      gfpfh.setOctreeLeafSize(this->m_params->template get<double>("gfpfh_octree_leafsize", model_resolution));
      gfpfh.setNumberOfClasses(this->m_params->template get<int>("gfpfh_number_of_classes", 16));
      gfpfh.setInputLabels(labels);
      // Compute the features
      gfpfh.compute(*features);

      // ?? !!
      features->sensor_orientation_ = cloud->sensor_orientation_;
      features->sensor_origin_ = cloud->sensor_origin_;

      if(this->m_context->exists("feature_stats"))
      {
        typename DataAccessor::Ptr stats = this->m_context->template get<typename DataAccessor::Ptr>("feature_stats");

        size_t c = pclDataLength<pcl::GFPFHSignature16>();
        for(size_t idx = 0; idx < features->size(); ++idx)
        {
          pcl::GFPFHSignature16 f = features->points[idx];
          float* d = pclData(f);
          for(size_t i = 0; i < c; ++i)
          {
            /*
            double min = stats->at(i, 0);
            double max = stats->at(i, 3);
            d[i] = (d[i] - min) / (max - min);
            */
            double mean = stats->at(i, 1);
            double var = stats->at(i, 2);
            d[i] = (d[i] - mean) / (std::sqrt(var));
          }
          features->points[idx] = f;
        }
      }

      this->m_context->set("features", features);

      double ms = (double(clock() - t0)*1000) / CLOCKS_PER_SEC;
      this->m_logger->set(row, 0, ms);
      this->m_logger->set(row, 1, features->size());

      return true;
    }

    std::string getName() const
    {
      return "VFH";
    }
  };
}

#endif
#endif
