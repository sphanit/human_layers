//Created by Phani Teja Singamaneni
#ifndef HUMAN_LAYERS_H
#define HUMAN_LAYERS_H
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <hanp_msgs/TrackedHumans.h>
#include <hanp_msgs/StateArray.h>
#include <hanp_msgs/TrackedSegmentType.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

namespace human_layers
{
class HumanLayer : public costmap_2d::Layer
{
public:
  HumanLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;

  virtual void updateBoundsFromHumans(double* min_x, double* min_y, double* max_x, double* max_y) = 0;

  bool isDiscretized()
  {
    return false;
  }

protected:
  struct HumanPoseVel{
    std_msgs::Header header;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist velocity;
  };

  void humansCB(const hanp_msgs::TrackedHumans& humans);

  void statesCB(const hanp_msgs::StateArray& states);

  double Guassian1D(double x, double x0, double A, double varx){
    double dx = x-x0;
    return A*exp(-pow(dx,2.0)/(2.0*varx));
  }

  double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary)
  {
    double dx = x - x0, dy = y - y0;
    double d = sqrt(dx * dx + dy * dy);
    double theta = atan2(dy, dx);
    double X = d*cos(theta), Y = d*sin(theta);
    return A/std::max(d,1.0) * Guassian1D(X,0.0,1.0,varx) * Guassian1D(Y,0.0,1.0,vary);
  }

  double Gaussian2D_skewed(double x, double y, double x0, double y0, double A, double varx, double vary, double skew_ang)
  {
    double dx = x - x0, dy = y - y0;
    double d = sqrt(dx * dx + dy * dy);
    double theta = atan2(dy, dx);
    double X = d*cos(theta-skew_ang), Y = d*sin(theta-skew_ang);
    return A/std::max(d,1.0) * Guassian1D(X,0.0,1.0,varx) * Guassian1D(Y,0.0,1.0,vary);
  }

  double getRadius(double cutoff, double A, double var)
  {
    return sqrt(-2 * var * log(cutoff / A));
  }

  ros::Subscriber humans_sub_, humans_states_sub_;
  hanp_msgs::TrackedHumans humans_;
  hanp_msgs::StateArray states_;
  std::vector<HumanPoseVel> transformed_humans_;
  boost::recursive_mutex lock_;
  bool first_time_, reset;
  ros::Time last_time;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double radius_, amplitude_, covar_, cutoff_;
  // double v_cutoff_, v_amplitude_, v_covar_;

};
}  // namespace human_layers

#endif  // HUMAN_LAYERS_H
