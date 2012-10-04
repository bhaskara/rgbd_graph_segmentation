/*
  Wrap the basic rgb library
*/

#include <cstdio>
#include <cstdlib>
#include <rgbd_graph_segmentation/rgbd_graph_segmentation.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <map>
#include <ros/assert.h>
#include "stl_tools.h"
#include "felz.h"

using std::vector;
using std::pair;
using std::map;
using std::set;
using std::cerr;



namespace rgbd_graph_segmentation
{

namespace sm=sensor_msgs;
namespace f=felzenszwalb;

typedef boost::shared_ptr<f::Img> ImgPtr;
typedef cv::Mat_<cv::Vec3b> RgbImage;
typedef cv::Mat_<float> DepthImage;
typedef std::map<Pixel, uint32_t> PixelMap;
typedef vector<Pixel> Segment;
typedef map<uint32_t, Segment> SegmentMap;

// Constructor for segmentation object just sets up the reverse mapping from
// segment id to pixel list
Segmentation::Segmentation (const RgbImage& image, const PixelMap& segs) :
  segments_(boost::extents[image.rows][image.cols]), image_(image)
{
  for (const PixelMap::value_type& e : segs)
  {
    // Add mapping from pixel to segment
    segments_[e.first.first][e.first.second] = e.second;
    
    // Add reverse mapping from segment id to pixel
    pixels_[e.second].push_back(e.first); // Creates map entry if doesn't exist
    segment_ids_.insert(e.second);
  }
}

// Constructor for segmentation object just sets up the reverse mapping from
// segment id to pixel list
Segmentation::Segmentation (const RgbImage& image, const array_type& segs) :
  segments_(segs), image_(image)
{
  for (int r=0; r<image_.rows; r++)
  {
    for (int c=0; c<image_.cols; c++)
    {
      const uint32_t seg = segs[r][c];
      pixels_[seg].push_back(Pixel(r, c));
      segment_ids_.insert(seg);
    }
  }
}


// Random color
cv::Vec3b randomColor ()
{
  cv::Vec3b v;
  v[0] = rand() % 256;
  v[1] = rand() % 256;
  v[2] = rand() % 256;
  return v;
}

// An assignment of colors to segment ids used for drawing borders
// Attempts to make the colors visually distinct
cv::Vec3b colorOfSegment (const uint32_t t)
{
  static map<uint32_t, cv::Vec3b> colors;
  static unsigned r=0, g=100, b=200;
  if (!contains(colors, t))
  {
    cv::Vec3b color;
    color[0] = b; 
    color[1] = g;
    color[2] = r;
    colors[t] = color;

    r = (r+91)%256;
    g = (g+51)%256;
    b = (b+71)%256;
  }
  return colors[t];
}

// Average rgb value in a given segment
cv::Vec3b
Segmentation::segmentAverageColor (const size_t i) const
{
  int r=0, g=0, b=0;
  const Segment& seg = pixels(i);
  const size_t n = seg.size();
  BOOST_FOREACH (const Pixel& p, seg)
  {
    const cv::Vec3b& color = image_(p.first, p.second);
    r += color[2];
    g += color[1];
    b += color[0];
  }
  cv::Vec3b avg;
  avg[2] = static_cast<float>(r)/n;
  avg[1] = static_cast<float>(g)/n;
  avg[0] = static_cast<float>(b)/n;
  return avg;
}

// Central pixel of a segment
Pixel Segmentation::center (const uint32_t i) const
{
  double r=0, c=0;
  const vector<Pixel>& pix = pixels(i);
  ROS_ASSERT(pix.size()>0);
  for (const Pixel& p : pix)
  {
    r += p.first;
    c += p.second;
  }
  return Pixel(r/pix.size(), c/pix.size());
}


// Return new image in which the border of each segment is highlighted a
// different color
RgbImage Segmentation::segmentationImage () const
{
  const uint32_t dummy_segment = -1;
  map<uint32_t, cv::Vec3b> colors;
  for (const uint32_t i : segment_ids_) 
  {
    if (i != dummy_segment)
      colors[i] = segmentAverageColor(i);
  }
  
  RgbImage img(image_.rows, image_.cols);
  cv::Vec3b orange;
  orange[2] = 255;
  orange[1] = 165;
  
  for (int r=0; r<image_.rows; r++)
  {
    for (int c=0; c<image_.cols; c++)
    {
      const uint32_t seg = segmentContaining(Pixel(r, c));
      const size_t n = pixels(seg).size();
      bool is_border = false;
      const int thickness = 3;
      for (int r2=r-thickness; r2<=r+thickness && !is_border; r2++)
      {
        if (r2<0 || r2>=image_.rows)
          continue;
        for (int c2=c-thickness; c2<=c+thickness && !is_border; c2++)
        {
          if (c2<0 || c2>=image_.cols)
            continue;
          if (segmentContaining(Pixel(r2, c2))!=seg)
            is_border = true;
        }
      }
      if (seg==dummy_segment || n<100)
        img(r, c) = image_(r, c);
      else if (is_border)
        img(r, c) = colorOfSegment(seg);
      else
        img(r, c) = image_(r, c); //colors.at(seg);
    }
  }

  for (const uint32_t i : segment_ids_)
  {
    if (i==dummy_segment)
      continue;
    const vector<Pixel>& pix = pixels(i);
    if (pix.size()<500)
      continue;
    Pixel p = center(i);
    for (int r=p.first-2; r<=p.first+2; r++)
    {
      for (int c=p.second-2; c<=p.second+2; c++)
      {
        if (r<0 || r>=image_.rows || c<0 || c>=image_.cols)
          continue;
        img(r, c) = orange;
      }
    }
  }

  
  return img;
}

// Return original image (note shares structure despite the const marking)
RgbImage Segmentation::image() const
{
  return image_;
}

const set<uint32_t> Segmentation::segmentIds () const
{
  return segment_ids_;
}


const vector<Pixel>& Segmentation::pixels (const uint32_t i) const
{
  return keyValue(pixels_, i);
}



// Convert from ROS to Felzenszwalb representation
ImgPtr convert (const RgbImage& image, const DepthImage& depth)
{
  ImgPtr res(new f::Img(image.cols, image.rows));
  f::Img* res_ptr = res.get();
  for (int r=0; r<image.rows; r++)
  {
    for (int c=0; c<image.cols; c++)
    {
      const cv::Vec3b& bgr = image(r, c);
      imRef(res_ptr, c, r).b = bgr[0];
      imRef(res_ptr, c, r).g = bgr[1];
      imRef(res_ptr, c, r).r = bgr[2];
      imRef(res_ptr, c, r).d = depth(r, c);
    }
  }
  return res;
}

Segmentation segment (const RgbImage& image,
                      const DepthImage& depth_image,
                      const float k, const unsigned min_size,
                      const float depth_threshold,
                      const float sigma)
{
  ROS_DEBUG_NAMED("felzenszwalb", "Converting image");
  ROS_ASSERT_MSG(image.rows==depth_image.rows && image.cols==depth_image.cols,
                 "Image dims (%d, %d) different from depth image dims (%d, %d)",
                 image.rows, image.cols, depth_image.rows, depth_image.cols);
  ImgPtr img = convert(image, depth_image);
  int num_comps;
  ROS_DEBUG_NAMED("felzenszwalb", "Segmenting");
  boost::shared_ptr<f::image<int> > comps(segment_image(img.get(), sigma, k,
                                                        min_size,
                                                        depth_threshold,
                                                        &num_comps));
  map<Pixel, uint32_t> res;
  f::image<int>* comps_ptr = comps.get();
  ROS_DEBUG_NAMED("felzenszwalb", "Converting segmentation");
  for (int x=0; x<comps->width(); x++) {
    for (int y=0; y<comps->height(); y++) {
      res[Pixel(y,x)] = imRef(comps_ptr, x, y);
    }
  }

  ROS_DEBUG_NAMED("felzenszwalb", "Indexing segmentation");
  Segmentation s(image, res);
  ROS_DEBUG_NAMED("felzenszwalb", "Segmentation complete");
  return s;
}

} // namespace rgbd_graph_segmentation
