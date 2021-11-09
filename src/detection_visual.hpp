/* Copyright 2021 Austrian Institute of Technology GmbH

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
#include <OgreSceneNode.h>
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/objects/covariance_visual.hpp"

namespace vision_rviz_plugins {

class DetectionVisual
{
private:
  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode * parent_scene_node_;
  Ogre::SceneNode * scene_node_;

  std::unique_ptr<rviz_rendering::Shape> bbox_;
  std::unique_ptr<rviz_rendering::CovarianceVisual> covariance_;

public:
  DetectionVisual(Ogre::SceneManager *, Ogre::SceneNode * parent_scene_node);

  DetectionVisual(DetectionVisual&&) = delete;

  virtual ~DetectionVisual();

  rviz_rendering::Shape & bbox();

  rviz_rendering::CovarianceVisual & covariance();

  void update(const vision_msgs::msg::Detection3D &);
};

} // namespace vision_rviz_plugins
