//Created by Phani Teja Singamaneni
#include <human_layers/human_layer.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#define DEFAULT_HUMAN_PART hanp_msgs::TrackedSegmentType::TORSO


using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace human_layers
{
void HumanLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  first_time_ = true;
  humans_sub_ = nh.subscribe("/tracked_humans", 1, &HumanLayer::humansCB, this);
  humans_states_sub_ = nh.subscribe("/move_base_node/TebLocalPlannerROS/humans_states", 1, &HumanLayer::statesCB, this);
}

void HumanLayer::humansCB(const hanp_msgs::TrackedHumans& humans)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  humans_ = humans;
}

void HumanLayer::statesCB(const hanp_msgs::StateArray& states){
  boost::recursive_mutex::scoped_lock lock(lock_);
  states_ = states;
}


void HumanLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  boost::recursive_mutex::scoped_lock lock(lock_);

  std::string global_frame = layered_costmap_->getGlobalFrameID();
  transformed_humans_.clear();

  for(auto &human : humans_.humans){
    for(auto &segment : human.segments){
      if(segment.type == DEFAULT_HUMAN_PART){
        if((abs(segment.twist.twist.linear.x)+abs(segment.twist.twist.linear.y)+abs(segment.twist.twist.angular.z)) < 0.0001 && !states_.states.empty()){
          if(states_.states[human.track_id-1]==0){
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
              tf_->transform(before_pose,after_pose,global_frame, ros::Duration(0.5));
              human_pose_vel.velocity.linear.x = after_pose.pose.position.x - human_pose_vel.pose.position.x;
              human_pose_vel.velocity.linear.y = after_pose.pose.position.y - human_pose_vel.pose.position.y;
              human_pose_vel.velocity.angular.z = angles::shortest_angular_distance(tf2::getYaw(after_pose.pose.orientation),tf2::getYaw(human_pose_vel.pose.orientation));

              transformed_humans_.push_back(human_pose_vel);
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

  updateBoundsFromHumans(min_x, min_y, max_x, max_y);
  if (first_time_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    first_time_ = false;
  }
  else
  {
    double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);
    last_min_x_ = a;
    last_min_y_ = b;
    last_max_x_ = c;
    last_max_y_ = d;
  }
}
};  // namespace human_layers
