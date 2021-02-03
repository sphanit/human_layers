//Created by Phani Teja Singamaneni
#include <human_layers/static_human_layer.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_eigen/tf2_eigen.h>
#define DEFAULT_HUMAN_PART hanp_msgs::TrackedSegmentType::TORSO

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace human_layers
{
class HumanVisibilityLayer : public StaticHumanLayer
{
public:
  HumanVisibilityLayer() : StaticHumanLayer() {}

void updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  boost::recursive_mutex::scoped_lock lock(lock_);

  std::string global_frame = layered_costmap_->getGlobalFrameID();
  transformed_humans_.clear();

  for(auto &human : humans_.humans){
    for(auto &segment : human.segments){
      if(segment.type == DEFAULT_HUMAN_PART){
        if((abs(segment.twist.twist.linear.x)+abs(segment.twist.twist.linear.y)+abs(segment.twist.twist.angular.z)) < 0.0001){
          HumanPoseVel human_pose_vel;
          human_pose_vel.header.frame_id = humans_.header.frame_id;
          human_pose_vel.header.stamp = humans_.header.stamp;
          geometry_msgs::PoseStamped before_pose, after_pose;

          try
          {
            before_pose.pose = segment.pose.pose;
            before_pose.header.frame_id = humans_.header.frame_id;
            before_pose.header.stamp = humans_.header.stamp;
            tf_->transform(before_pose,after_pose,global_frame,ros::Duration(0.5));
            human_pose_vel.pose = after_pose.pose;

            before_pose.pose.position.x += segment.twist.twist.linear.x;
            before_pose.pose.position.y += segment.twist.twist.linear.y;
            auto hb_yaw = tf2::getYaw(before_pose.pose.orientation);
            tf2::Quaternion quat;
            quat.setEuler(segment.twist.twist.angular.z + hb_yaw,0.0,0.0);
            tf2::convert(before_pose.pose.orientation, quat);
            tf_->transform(before_pose,after_pose,global_frame,ros::Duration(0.5));
            human_pose_vel.velocity.linear.x = after_pose.pose.position.x - human_pose_vel.pose.position.x;
            human_pose_vel.velocity.linear.y = after_pose.pose.position.y - human_pose_vel.pose.position.y;
            human_pose_vel.velocity.angular.z = angles::shortest_angular_distance(tf2::getYaw(after_pose.pose.orientation),tf2::getYaw(human_pose_vel.pose.orientation));

            transformed_humans_.push_back(human_pose_vel);

            double mag = sqrt(pow(human_pose_vel.velocity.linear.x, 2) + pow(human_pose_vel.velocity.linear.y, 2));
            double factor = 1.0 + mag * factor_;
            double point = get_radius(v_cutoff_, v_amplitude_, v_covar_ * factor);

            *min_x = std::min(*min_x, human_pose_vel.pose.position.x - point);
            *min_y = std::min(*min_y, human_pose_vel.pose.position.y - point);
            *max_x = std::max(*max_x, human_pose_vel.pose.position.x + point);
            *max_y = std::max(*max_y, human_pose_vel.pose.position.y + point);

          }
          catch (tf2::LookupException& ex)
          {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            continue;
          }
          catch (tf2::ConnectivityException& ex)
          {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            continue;
          }
          catch (tf2::ExtrapolationException& ex)
          {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            continue;
          }
        }
      }
    }
  }
}

virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_) return;

  if (humans_.humans.size() == 0)
    return;
  if (v_cutoff_ >= v_amplitude_)
    return;

  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  for(uint i=0;i<transformed_humans_.size();i++){
    auto human = transformed_humans_[i];
    double theta = tf2::getYaw(human.pose.orientation);
    Eigen::Vector2d orient_vec(std::cos(theta),std::sin(theta));
    double angle = atan2(human.velocity.linear.y, human.velocity.linear.x);
    double mag = sqrt(pow(human.velocity.linear.x, 2) + pow(human.velocity.linear.y, 2));
    double factor = 1.0 + mag * factor_;
    double base = get_radius(v_cutoff_, v_amplitude_, v_covar_);
    double point = get_radius(v_cutoff_, v_amplitude_, v_covar_ * factor);

    unsigned int width = std::max(1, static_cast<int>((base + point) / res)),
                 height = std::max(1, static_cast<int>((base + point) / res));

    double cx = human.pose.position.x, cy = human.pose.position.y;

    double ox, oy;
    if (sin(angle) > 0)
      oy = cy - base;
    else
      oy = cy + (point - base) * sin(angle) - base;

    if (cos(angle) >= 0)
      ox = cx - base;
    else
      ox = cx + (point - base) * cos(angle) - base;


    int dx, dy;
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    int start_x = 0, start_y = 0, end_x = width, end_y = height;
    if (dx < 0)
      start_x = -dx;
    else if (dx + width > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i)
      start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)
      end_x = max_i - dx;

    if (dy < 0)
      start_y = -dy;
    else if (dy + height > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

    if (static_cast<int>(start_y + dy) < min_j)
      start_y = min_j - dy;
    if (static_cast<int>(end_y + dy) > max_j)
      end_y = max_j - dy;

    double bx = ox + res / 2,
           by = oy + res / 2;

    for (int i = start_x; i < end_x; i++)
    {
      for (int j = start_y; j < end_y; j++)
      {
        unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        if (old_cost == costmap_2d::NO_INFORMATION)
          continue;

        double x = bx + i * res, y = by + j * res;
        double ma = atan2(y - cy, x - cx);
        double diff = angles::shortest_angular_distance(angle, ma);
        double a;
        if (fabs(diff) < M_PI / 2)
          a = gaussian(x, y, cx, cy, v_amplitude_, v_covar_ * factor, v_covar_, angle);
        else
          a = gaussian(x, y, cx, cy, v_amplitude_, v_covar_, v_covar_, 0);

        Eigen::Vector2d pt_vec(x-cx,y-cy);

        if (a < v_cutoff_)
          continue;
        unsigned char cvalue = (unsigned char) std::min(10*a,254.0);
        if(orient_vec.dot(pt_vec) <= 0)
          costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
      }
    }
  }
}

};
};  // namespace human_layers

PLUGINLIB_EXPORT_CLASS(human_layers::HumanVisibilityLayer, costmap_2d::Layer)
