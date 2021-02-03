//Created by Phani Teja Singamaneni
#ifndef STATIC_HUMAN_LAYER_H
#define STATIC_HUMAN_LAYER_H
#include <ros/ros.h>
#include <human_layers/human_layer.h>
#include <dynamic_reconfigure/server.h>
#include <human_layers/HumanLayerConfig.h>

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);

namespace human_layers
{
class StaticHumanLayer : public HumanLayer
{
public:
  StaticHumanLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBoundsFromHumans(double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  void configure(HumanLayerConfig &config, uint32_t level);
  double cutoff_, amplitude_, covar_, factor_;
  double v_cutoff_, v_amplitude_, v_covar_;
  dynamic_reconfigure::Server<HumanLayerConfig>* server_;
  dynamic_reconfigure::Server<HumanLayerConfig>::CallbackType f_;
};
}  // namespace human_layers

#endif  // STATIC_HUMAN_LAYER_H
