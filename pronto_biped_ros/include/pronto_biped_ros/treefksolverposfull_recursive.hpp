// Copyright  (C)  2009 Willow Garage Inc

// Version: 1.0
// Author: Wim Meeussen <meeussen at willowgarage dot com>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef KDLTREEFKSOLVERPOSFULL_RECURSIVE_HPP
#define KDLTREEFKSOLVERPOSFULL_RECURSIVE_HPP

#include <kdl/tree.hpp>

namespace KDL {

class TreeFkSolverPosFull_recursive

{
public:
  TreeFkSolverPosFull_recursive(const Tree& _tree);
  ~TreeFkSolverPosFull_recursive();

  int JntToCart(const std::map<std::string, double>& q_in, std::map<std::string, KDL::Frame>& p_out, bool flatten_tree=true);
 
private:
   
  void addFrameToMap(const std::map<std::string, double>& q_in, 
	     std::map<std::string, KDL::Frame >& p_out,
	     const KDL::Frame& previous_frame,
	     const SegmentMap::const_iterator this_segment,
	     bool flatten_tree);

  Tree tree;

};
}

#endif
