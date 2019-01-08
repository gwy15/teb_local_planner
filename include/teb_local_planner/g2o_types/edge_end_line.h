/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,
 *  Weiyu Guan - Tsinghua University.
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
 *   * Neither the name of the institute nor the names of its
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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Weiyu Guan
 *********************************************************************/

#ifndef EDGE_END_LINE_H_
#define EDGE_END_LINE_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include "g2o/core/base_unary_edge.h"


namespace teb_local_planner
{

/**
 * @class EdgeEndLine
 * @brief Edge defining the cost function for constraining a configuration on a ending line
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min  dist2line \cdot weight \f$. \n
 * \e dist2line denotes the distance to the end line. \n
 * \e weight can be set using setInformation(). \n
 * @see TebOptimalPlanner::AddEdgesEndLine
 * @remarks Do not forget to call setTebConfig() and setEndLine()
 */     
class EdgeEndLine : public BaseTebUnaryEdge<1, const std::tuple<Eigen::Vector2d, Eigen::Vector2d>*, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgeEndLine() 
  {
    _measurement = NULL;
  }
 
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig(), setEndLine() on EdgeEndLine()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    Eigen::Vector2d p = bandpt->position();
    Eigen::Vector2d p1 = std::get<0>(*_measurement);
    Eigen::Vector2d p2 = std::get<1>(*_measurement);
    _error[0] = dist2segment(p.x(), p.y(), p1.x(), p1.y(), p2.x(), p2.y());

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeEndLine::computeError() _error[0]=%f\n",_error[0]);
  }

  double dist2segment(double x, double y, double x1, double y1, double x2, double y2) {
    double A = x - x1,  B = y - y1;
    double C = x2 - x1, D = y2 - y1;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double ratio = -1;
    if (len_sq != 0) //in case of 0 length line
      ratio = dot / len_sq;

    double xx, yy;

    if (ratio < 0) {
      xx = x1;
      yy = y1;
    }
    else if (ratio > 1) {
      xx = x2;
      yy = y2;
    }
    else {
      xx = x1 + ratio * C;
      yy = y1 + ratio * D;
    }

    double dx = x - xx, dy = y - yy;
    return std::sqrt(dx * dx + dy * dy);
  }

  /**
   * @brief Set pointer to associated end line for the underlying cost function 
   * @param line a pointer to std::tuple<vec, vec> containing the position of the two points
   */ 
  void setEndLine(const std::tuple<Eigen::Vector2d, Eigen::Vector2d>* line)
  {
    _measurement = line;
  }
    
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param line a pointer to std::tuple<vec, vec> containing the position of the two points
   */ 
  void setParameters(const TebConfig& cfg, const std::tuple<Eigen::Vector2d, Eigen::Vector2d>* line)
  {
    cfg_ = &cfg;
    _measurement = line;
  }
  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
  
    

} // end namespace

#endif
