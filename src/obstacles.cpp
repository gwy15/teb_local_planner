/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/obstacles.h>
#include <ros/console.h>
#include <ros/assert.h>
// #include <teb_local_planner/misc.h>

#include <chrono>
class SpeedCounter {
  std::string _name;
  std::chrono::high_resolution_clock _timer;
  decltype(_timer.now()) _nextTimeToLog;
  long long _count = 0;
  const decltype(std::chrono::seconds(1)) _1s = std::chrono::seconds(1);
public:
  SpeedCounter(std::string name): _timer(), _count(0), _name(name) {
    _nextTimeToLog = _timer.now() + _1s;
  }
  virtual ~SpeedCounter(){}

  void count() {
    _count += 1;
    if (_timer.now() > _nextTimeToLog) {
      _nextTimeToLog += _1s;
      std::cout << "Counter " << _name << " was invoked " << _count << " times in 1s." << std::endl;
      _count = 0;
    }
  }
};

namespace teb_local_planner
{
auto totalCounter = SpeedCounter("total counter");
auto calcCounter = SpeedCounter("calc counter");

using _Type = const geometry_msgs::PoseStamped &;
double calcR(_Type p1, _Type p2, _Type p3) {
  double x1 = p1.pose.position.x, y1 = p1.pose.position.y;
  double x2 = p2.pose.position.x, y2 = p2.pose.position.y;
  double x3 = p3.pose.position.x, y3 = p3.pose.position.y;

  double a = x1 - x2, b =  y1 - y2, c = x1 - x3, d = y1 - y3;
  double e = ((x1 * x1 - x2 * x2) - (y2 * y2 - y1 * y1)) / 2.0;
  double f = ((x1 * x1 - x3 * x3) - (y3 * y3 - y1 * y1)) / 2.0;
  double x0 = - (d * e - b * f) / (b * c - a * d);
  double y0 = - (a * f - c * e) / (b * c - a * d);

  double r = std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
  return r;
}

void calcRho(PathPtr path, std::vector<double>& rho) {
  const auto& begin = path->begin();
  const auto& last = (--path->end());
  
  double r = 0.0;
  for (auto iter = path->begin(); iter != path->end(); iter++) {
    if (iter == begin || iter == last) {
      r = 10.0; // ignore the first and last one
    } else {
      r = calcR(*(iter-1), *iter, *(iter+1));
    }
    rho.push_back(r);
  }
}

inline Eigen::Vector2d Pose2Vec2d(const geometry_msgs::PoseStamped& pose) {
  return Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
}

inline double getIdealTangentialVelocityWithRho(const double rho) {
  constexpr const double vm = 5.0, v1 = 2.5;
  constexpr const double ratio = vm / v1;
  const double _temp = 1 + (std::pow(ratio, 4.0) - 1) / (rho * rho);
  return vm / std::pow(_temp, 0.25);
}

Eigen::Vector2d PointObstacle::estimatePoseAtTime(double t) const {
  // static obstacle
  if (!dynamic_ && !racer_ ) {
    return pos_;
  }
  // dynamic obstacle, but not racer
  else if (dynamic_ && !racer_) {
    return pos_ + t * centroid_velocity_;
  }
  // racer obstacles
  // put this block in a else statement to avoid extra memory allocate
  else {
    // counter for racer obstacle
    totalCounter.count();

    // if found stashed estimation, return directly.
    auto && findResult = estimatedPos_.find(t);
    if (findResult != estimatedPos_.end()) {
      return findResult->second;
    }

    // counter for re-calculated obstacle
    calcCounter.count();

    // ============== find the closest first pose as start pose ===========
    std::size_t startIndex = 0;
    Eigen::Vector2d startPose;
    { // for loop to find start pose
      auto index = 0;
      for (; index<_initPath->size()-1; index++) {
        auto && p0 = Pose2Vec2d(_initPath->at(index));
        auto && p1 = Pose2Vec2d(_initPath->at(index+1));
        auto _cos = (p1 - p0).dot(pos_ - p0);
        if (_cos > 0) {
          continue;
        } else {
          startIndex = index;
          startPose = p0;
          break;
        }
      }
      // failed to find start pose, fallback to constant velocity.
      if (index == _initPath->size()-2) {
        return pos_ + t * centroid_velocity_;
      }
    }
    // ================== output: startIndex, startPose ===================

    // ======================== normal direction ==========================
    // since (p1-p0).(pos_-p0) is near 0, we can use |startPose-p0| as d_n
    auto && poseToStart  = pos_ - startPose; // cache to deduce calculation
    auto && d_n_0 = poseToStart.norm();
    // projection of v on direction poseToStart
    auto && v_n_0 = poseToStart.dot(centroid_velocity_) / d_n_0;
    {// calc direction
      auto && p = Pose2Vec2d(_initPath->at(startIndex + 1)) - startPose;
      const auto & v = centroid_velocity_;
      v_n_0 *= p[0] * v[1] - p[1] * v[0] > 0 ? 1.0: -1.0;
    }
    // =================== output: d_n_0 and v_n_0 =======================
    
    // ======================= tangent direction =========================
    double v_tau_0;
    { // calculate v_tau
      auto && p0 = startPose;
      auto && p1 = Pose2Vec2d(_initPath->at(startIndex + 1));
      auto && p10 = p1 - p0;
      v_tau_0 = centroid_velocity_.dot(p10) / p10.norm();
    }
    double v_tau_ideal_0 = getIdealTangentialVelocityWithRho(_rho.at(startIndex));
    double delta_v_tau_0 = v_tau_0 - v_tau_ideal_0;

    const double t_tau = 3.0; // TODO:

    double tm = 0.0; // time for loop
    std::size_t finalIndex = startIndex;
    double v_tau = 0.0; // v_tau

    for (auto i=startIndex; i<_initPath->size()-1; i++) {
      if (i == _initPath->size() - 2) {
        // moving out of path
        return pos_ + t * centroid_velocity_;
      }
      auto && p0 = Pose2Vec2d(_initPath->at(i));
      auto && p1 = Pose2Vec2d(_initPath->at(i+1));
      
      auto rho = _rho.at(i);
      auto v_tau_ideal = getIdealTangentialVelocityWithRho(rho);
      v_tau = v_tau_ideal + delta_v_tau_0 * std::exp(-tm / t_tau);

      tm += (p1 - p0).norm() / v_tau;
      if (tm > t) {
        finalIndex = i;
        break;
      }
    }

    // Calculate final pose
    Eigen::Vector2d pose;
    {
      constexpr const double omega_n = 2.0;
      // TODO:
      auto basePose = Pose2Vec2d(_initPath->at(finalIndex));
      auto baseDirection = Pose2Vec2d(_initPath->at(finalIndex + 1));
      double expFactor = std::exp(-omega_n * t);
      double d_n = (d_n_0 + (v_n_0 + omega_n * d_n_0) * t) * expFactor;
      double v_n = (v_n_0 - (v_n_0 + omega_n * d_n_0) * omega_n * t) * expFactor;

      pose = pos_ + t * centroid_velocity_;
    }

    // stash data for further use.
    const_cast<PointObstacle*>(this)->estimatedPos_.insert({t, pose});
    return pose;
  }
}


void PolygonObstacle::fixPolygonClosure()
{
  if (vertices_.size()<2)
    return;
  
  if (vertices_.front().isApprox(vertices_.back()))
    vertices_.pop_back();
}

void PolygonObstacle::calcCentroid()
{
  if (vertices_.empty())
  {
    centroid_.setConstant(NAN);
    ROS_WARN("PolygonObstacle::calcCentroid(): number of vertices is empty. the resulting centroid is a vector of NANs.");
    return;
  }
  
  // if polygon is a point
  if (noVertices()==1)
  {
    centroid_ = vertices_.front();
    return;
  }
  
  // if polygon is a line:
  if (noVertices()==2)
  {
    centroid_ = 0.5*(vertices_.front() + vertices_.back());
    return;
  }
  
  // otherwise:
  
  centroid_.setZero();
    
  // calculate centroid (see wikipedia http://de.wikipedia.org/wiki/Geometrischer_Schwerpunkt#Polygon)
  double A = 0;  // A = 0.5 * sum_0_n-1 (x_i * y_{i+1} - x_{i+1} * y_i)
  for (int i=0; i < noVertices()-1; ++i)
  {
    A += vertices_.at(i).coeffRef(0) * vertices_.at(i+1).coeffRef(1) - vertices_.at(i+1).coeffRef(0) * vertices_.at(i).coeffRef(1);
  }
  A += vertices_.at(noVertices()-1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices()-1).coeffRef(1);
  A *= 0.5;
  
  if (A!=0)
  {
    for (int i=0; i < noVertices()-1; ++i)
    {
      double aux = (vertices_.at(i).coeffRef(0) * vertices_.at(i+1).coeffRef(1) - vertices_.at(i+1).coeffRef(0) * vertices_.at(i).coeffRef(1));
      centroid_ +=  ( vertices_.at(i) + vertices_.at(i+1) )*aux;
    }
    double aux = (vertices_.at(noVertices()-1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices()-1).coeffRef(1));
    centroid_ +=  ( vertices_.at(noVertices()-1) + vertices_.at(0) )*aux;
    centroid_ /= (6*A);	
  }
  else // A == 0 -> all points are placed on a 'perfect' line
  {
    // seach for the two outer points of the line with the maximum distance inbetween
    int i_cand = 0;
    int j_cand = 0;
    double max_dist = 0;
    for (int i=0; i< noVertices(); ++i)
    {
      for (int j=i+1; j< noVertices(); ++j) // start with j=i+1
      {
        double dist = (vertices_[j] - vertices_[i]).norm();
        if (dist > max_dist)
        {
          max_dist = dist;
          i_cand = i;
          j_cand = j;
        }
      }
    }
    // calc centroid of that line
    centroid_ = 0.5*(vertices_[i_cand] + vertices_[j_cand]);
  }
}



Eigen::Vector2d PolygonObstacle::getClosestPoint(const Eigen::Vector2d& position) const
{
  // the polygon is a point
  if (noVertices() == 1)
  {
    return vertices_.front();
  }
  
  if (noVertices() > 1)
  {
    
    Eigen::Vector2d new_pt = closest_point_on_line_segment_2d(position, vertices_.at(0), vertices_.at(1));
    
    if (noVertices() > 2) // real polygon and not a line
    {
      double dist = (new_pt-position).norm();
      Eigen::Vector2d closest_pt = new_pt;
      
      // check each polygon edge
      for (int i=1; i<noVertices()-1; ++i) // skip the first one, since we already checked it (new_pt)
      {
        new_pt = closest_point_on_line_segment_2d(position, vertices_.at(i), vertices_.at(i+1));
        double new_dist = (new_pt-position).norm();
        if (new_dist < dist)
        {
          dist = new_dist;
          closest_pt = new_pt;
        }
      }
      // afterwards we check the edge between goal and start (close polygon)
      new_pt = closest_point_on_line_segment_2d(position, vertices_.back(), vertices_.front());
      double new_dist = (new_pt-position).norm();
      if (new_dist < dist)
        return new_pt;
      else
        return closest_pt;
    }
    else
    {
      return new_pt; // closest point on line segment
    }
  }

  ROS_ERROR("PolygonObstacle::getClosestPoint() cannot find any closest point. Polygon ill-defined?");
  return Eigen::Vector2d::Zero(); // todo: maybe boost::optional?
}


bool PolygonObstacle::checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist) const
{
  // Simple strategy, check all edge-line intersections until an intersection is found...
  // check each polygon edge
  for (int i=0; i<noVertices()-1; ++i)
  {
    if ( check_line_segments_intersection_2d(line_start, line_end, vertices_.at(i), vertices_.at(i+1)) ) 
      return true;
  }
  if (noVertices()==2) // if polygon is a line
    return false;
  
  return check_line_segments_intersection_2d(line_start, line_end, vertices_.back(), vertices_.front()); //otherwise close polygon
}



// implements toPolygonMsg() of the base class
void PolygonObstacle::toPolygonMsg(geometry_msgs::Polygon& polygon)
{
  polygon.points.resize(vertices_.size());
  for (std::size_t i=0; i<vertices_.size(); ++i)
  {
    polygon.points[i].x = vertices_[i].x();
    polygon.points[i].y = vertices_[i].y();
    polygon.points[i].z = 0;
  }
}








} // namespace teb_local_planner
