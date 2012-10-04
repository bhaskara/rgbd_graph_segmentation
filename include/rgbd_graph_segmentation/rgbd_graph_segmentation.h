/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * Wrapper for the Felzenszwalb segmenter
 *
 * \author Bhaskara Marthi
 */

#ifndef RGBD_GRAPH_SEGMENTATION_H
#define RGBD_GRAPH_SEGMENTATION_H

#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <set>
#include <boost/multi_array.hpp>

namespace rgbd_graph_segmentation
{

// Coordinates of an individual pixel, in order r, c
// TODO: just use cv::Point
typedef std::pair<uint16_t, uint16_t> Pixel;

// An object that represents a segmentation of an image
class Segmentation
{
public:
  
  typedef boost::multi_array<uint32_t, 2> array_type;

  Segmentation(const cv::Mat_<cv::Vec3b>& image,
               const std::map<Pixel, uint32_t>& segs);
  
  Segmentation (const cv::Mat_<cv::Vec3b>& image,
                const array_type& segs);

  // Which segment a given pixel belongs to
  inline
  uint32_t segmentContaining (const Pixel& p) const
  {
    return segments_[p.first][p.second];
  };

  // List of pixels in a segment
  const std::vector<Pixel>& pixels (uint32_t seg) const;
  
  // Get an image where each segment is assigned a random color
  cv::Mat_<cv::Vec3b> segmentationImage() const;
  
  // Get the original image
  cv::Mat_<cv::Vec3b> image() const;
  
  // List of segments
  const std::set<uint32_t> segmentIds() const;

  // Center (mean) of a segment
  Pixel center (uint32_t seg) const;
  
private:
  
  cv::Vec3b segmentAverageColor(size_t i) const;

  array_type segments_;
  cv::Mat_<cv::Vec3b> image_;
  std::set<uint32_t> segment_ids_;
  std::map<uint32_t, std::vector<Pixel> > pixels_;
};

// Call Felzenszwalb's segmenter with the given parameters.
// sigma is the initial smoothing width, the k parameter governs how likely 
// things are to get combined during clustering (higher values mean larger
// clusters), and min_size also governs cluster size, as a postprocessing step.
// Edges will not be added between adjacent pixels whose depth difference
// exceeds the depth threshold.
Segmentation segment (const cv::Mat_<cv::Vec3b>& image,
                      const cv::Mat_<float>& depth_image,
                      float k = 1000,
                      unsigned min_size = 500,
                      float depth_threshold=.01,
                      float sigma = 0.5);


} // namespace


#endif // include guard
