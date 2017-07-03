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
#ifndef PCLREF_CDF_HPP_
#define PCLREF_CDF_HPP_
#include <CDF.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/exceptions.h>
#include <pcl/impl/pcl_base.hpp>

namespace pclref
{
/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT>
  CDF<PointT>::CDF (const PointCloud &cloud)
  {
    Base ();
    setInputCloud (cloud.makeShared ());
  }

  template<typename PointT> void
  CDF<PointT>::compute ()
  {
    if (!compute_done_)
      initCompute ();

    assert(samples_ > 1);

    const size_t dataLength = pclref::pclDataLength<PointT>();
    const size_t pointCount = indices_->size();
    std::vector<std::vector<float> > rows;

    rows.resize(dataLength);
    cdf_.resize(dataLength);
    for(size_t i = 0; i < dataLength; ++i)
    {
      cdf_[i].resize(samples_);
      rows[i].resize(pointCount);
    }
    matrix_.resize(samples_, dataLength);

    for(size_t i = 0; i < indices_->size(); ++i)
    {
      int idx = (*indices_)[i];
      if(!isValid((*input_)[idx]))
        continue;
      const float* p = pclData((*input_)[idx]);
      int j = 0;
      for(size_t j = 0; j < dataLength; ++j)
      {
        rows[j][i] = p[j];
      }
    }

    for(size_t i = 0; i < dataLength; ++i)
    {
      std::vector<float> row = rows[i];
      std::sort(row.begin(), row.end());
      float step = ((float)row.size()-1) / (samples_-1);
      for(size_t j = 0; j < samples_; ++j)
      {
        size_t idx = (size_t)(j * step);
        cdf_[i][j] = row[idx];
        matrix_(j, i) = row[idx];
      }
    }

    compute_done_ = true;
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> bool
  CDF<PointT>::initCompute ()
  {
    if(!Base::initCompute ())
    {
      return (false);
    }
    return (true);
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  CDF<PointT>::project (const PointT& input, PointT& projection)
  {
    if(!compute_done_)
      compute ();

    const size_t dataLength = pclref::pclDataLength<PointT>();
    float* data = pclref::pclData(projection);
    const float* in = pclref::pclData(input);
    for(size_t i = 0; i < dataLength; ++i)
    {
      std::vector<float> row = cdf_[i];
      std::vector<float>::iterator hit = std::upper_bound(row.begin(), row.end(), in[i]);
      std::size_t index = hit - row.begin();

      if(hit == row.begin())
        data[i] = 0.0f;
      else if(hit == row.end())
        data[i] = 1.0f;
      else
      {
        float a = *(hit-1);
        float b = *(hit);
        data[i] = in[i] - a;
        data[i] /= ((b-a) * samples_);
        data[i] += ((float)index - 1) / samples_;
      }
    }
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  CDF<PointT>::project (const PointCloud& input, PointCloud& projection)
  {
    if(!compute_done_)
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
  CDF<PointT>::reconstruct (const PointT& projection, PointT& input)
  {
    if(!compute_done_)
      compute ();

    const size_t dataLength = pclref::pclDataLength<PointT>();
    float* data = pclref::pclData(input);
    const float* in = pclref::pclData(projection);
    for(size_t i = 0; i < dataLength; ++i)
    {
      std::vector<float> row = cdf_[i];
      float v = in[i];
      int offset = (int)(v*samples_);
      float t = v*samples_ - offset;

      if(offset > 0)
      {
        data[i] = t*row[offset-1] + (1-t)*row[offset];
      }
      else
      {
        data[i] = row[offset];
      }
    }
  }

/////////////////////////////////////////////////////////////////////////////////////////
  template<typename PointT> void
  CDF<PointT>::reconstruct (const PointCloud& projection, PointCloud& input)
  {
    if(!compute_done_)
      compute ();
    if (!compute_done_)
      PCL_THROW_EXCEPTION (InitFailedException, "[pcl::PCA::reconstruct] PCA initCompute failed");
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
