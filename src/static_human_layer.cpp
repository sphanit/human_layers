//Created by Phani Teja Singamaneni
#include <human_layers/static_human_layer.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_eigen/tf2_eigen.h>

PLUGINLIB_EXPORT_CLASS(human_layers::StaticHumanLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew)
{
  double dx = x - x0, dy = y - y0;
  double h = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double mx = cos(angle - skew) * h;
  double my = sin(angle - skew) * h;
  double f1 = pow(mx, 2.0) / (2.0 * varx),
         f2 = pow(my, 2.0) / (2.0 * vary);
  return A * exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var)
{
  return sqrt(-2 * var * log(cutoff / A));
}


namespace human_layers
{
void StaticHumanLayer::onInitialize()
{
  HumanLayer::onInitialize();
  ros::NodeHandle nh("~/" + name_), g_nh;
  server_ = new dynamic_reconfigure::Server<HumanLayerConfig>(nh);
  f_ = boost::bind(&StaticHumanLayer::configure, this, _1, _2);
  server_->setCallback(f_);
}

void StaticHumanLayer::updateBoundsFromHumans(double* min_x, double* min_y, double* max_x, double* max_y)
{

  for(uint i=0;i<transformed_humans_.size();i++){
    double mag = sqrt(pow(transformed_humans_[i].velocity.linear.x, 2) + pow(transformed_humans_[i].velocity.linear.y, 2));
    double factor = 1.0 + mag * factor_;
    double point = get_radius(cutoff_, amplitude_, covar_ * factor);

    auto human = transformed_humans_[i];
    *min_x = std::min(*min_x, human.pose.position.x - point);
    *min_y = std::min(*min_y, human.pose.position.y - point);
    *max_x = std::max(*max_x, human.pose.position.x + point);
    *max_y = std::max(*max_y, human.pose.position.y + point);
  }
}

void StaticHumanLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (!enabled_) return;

  if (humans_.humans.size() == 0)
    return;
  if (cutoff_ >= amplitude_)
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
    double base = get_radius(cutoff_, amplitude_, covar_);
    double point = get_radius(cutoff_, amplitude_, covar_ * factor);

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
          a = gaussian(x, y, cx, cy, amplitude_, covar_ * factor, covar_, angle);

        else
          a = gaussian(x, y, cx, cy, amplitude_, covar_, covar_, 0);

        Eigen::Vector2d pt_vec(x-cx,y-cy);

        if (a < cutoff_)
          continue;
        unsigned char cvalue = (unsigned char) a;
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
      }
    }
  }
}

void StaticHumanLayer::configure(HumanLayerConfig &config, uint32_t level)
{
  cutoff_ = config.cutoff;
  amplitude_ = config.amplitude;
  covar_ = config.covariance;
  v_cutoff_ = config.visibility_cutoff;
  v_amplitude_ = config.visibility_amplitude;
  v_covar_ = config.visibility_covariance;
  factor_ = config.factor;
  people_keep_time_ = ros::Duration(config.keep_time);
  enabled_ = config.enabled;
}
};  // namespace human_layers
