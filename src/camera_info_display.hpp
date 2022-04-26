/* Copyright 2022 Austrian Institute of Technology GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#pragma once

#include <memory>
#include <list>
#include <utility>
#include "rviz_common/message_filter_display.hpp"
#include "camera_info_visual.hpp"

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
}
}

namespace vision_rviz_plugins
{

using sensor_msgs::msg::CameraInfo;

class CameraInfoDisplay : public
  rviz_common::MessageFilterDisplay<CameraInfo>
{
  Q_OBJECT

public:
  CameraInfoDisplay();

  ~CameraInfoDisplay() override;

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateFarDistance();

private:
  void processMessage(CameraInfo::ConstSharedPtr message) override;

  std::unique_ptr<CameraInfoVisual> visual_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::FloatProperty * far_distance_property_;
};

}  // namespace vision_rviz_plugins
