//Created by Phani Teja Singamaneni
#ifndef HUMAN_VISIBILITY_LAYER_H
#define HUMAN_VISIBILITY_LAYER_H
#include <ros/ros.h>
#include <human_layers/human_layer.h>
#include <dynamic_reconfigure/server.h>
#include <human_layers/HumanVLayerConfig.h>

namespace human_layers
{
class HumanVisibilityLayer : public HumanLayer
{
public:
  HumanVisibilityLayer()
  {
    layered_costmap_ = NULL;
  }

  virtual void onInitialize();
  virtual void updateBoundsFromHumans(double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

protected:
  void configure(HumanVLayerConfig &config, uint32_t level);
  dynamic_reconfigure::Server<HumanVLayerConfig>* server_;
  dynamic_reconfigure::Server<HumanVLayerConfig>::CallbackType f_;
};
}  // namespace human_layers

#endif  // HUMAN_VISIBILITY_LAYER_H
