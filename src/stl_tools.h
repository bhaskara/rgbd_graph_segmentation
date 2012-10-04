
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
 * STL and boost related utils
 *
 * \author Bhaskara Marthi
 */

#ifndef RGBD_GRAPH_SEGMENTATION_TOOLS_STL_TOOLS_H
#define RGBD_GRAPH_SEGMENTATION_TOOLS_STL_TOOLS_H

#include <map>
#include <boost/function.hpp>

namespace rgbd_graph_segmentation
{

// Get value corresponding to a key in a map
// Throw std::out_of_range if it's not there
template <class K, class V, class C, class A>
const V& keyValue (const std::map<K, V, C, A>& m, const K& key)
{
  typename std::map<K, V, C, A>::const_iterator pos = m.find(key);
  if (pos==m.end())
    throw std::out_of_range("Map did not contain key it was expected to");
  return pos->second;
}

// Does the given container contain the given element?
template <class C, class K>
bool contains (const C& container, const K& key)
{
  return container.find(key)!=container.end();
}

// Does any element of the container satisfy the predicate?
template <class Container, class Pred>
bool any (const Container& c, const Pred& p)
{
  return (find_if(c.begin(), c.end(), p)!=c.end());
}
          

// A lazily computed map, aka a memoized function
template <class K, class V>
class MemoizedFunction
{
public:

  typedef boost::function<V (const K& k)> fn_type;

  MemoizedFunction (fn_type f) : func(f) {};

  V operator() (const K& k)
  {
    typename std::map<K, V>::const_iterator pos = map_.find(k);
    if (pos==map_.end())
      map_[k] = func(k);
    return keyValue(map_, k);
  }
  
private:

  fn_type func;
  std::map<K, V> map_;
};


} // namespace



#endif // include guard
