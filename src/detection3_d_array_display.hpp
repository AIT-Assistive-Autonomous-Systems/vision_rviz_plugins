#pragma once

#include <memory>
#include <vector>
#include <utility>
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/objects/covariance_visual.hpp"
#include "rviz_common/message_filter_display.hpp"

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
class CovarianceProperty;
}
}

namespace vision_rviz_plugins
{

class Detection3DArrayDisplay : public
  rviz_common::MessageFilterDisplay<vision_msgs::msg::Detection3DArray>
{
  Q_OBJECT

public:
  Detection3DArrayDisplay();

  ~Detection3DArrayDisplay() override;

  void onInitialize() override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateCovariance();

private:
  void processMessage(vision_msgs::msg::Detection3DArray::ConstSharedPtr message) override;

  std::vector<rviz_rendering::Shape> detections_;
  std::vector<rviz_rendering::CovarianceVisual> covariances_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::CovarianceProperty * covariance_property_;
};

}  // namespace vision_rviz_plugins
