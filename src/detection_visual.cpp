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

#include "detection_visual.hpp"
#include "rviz_common/msg_conversions.hpp"
#include <utility>
#include <memory>

namespace vision_rviz_plugins
{

using rviz_rendering::Shape;
using rviz_rendering::CovarianceVisual;

DetectionVisual::DetectionVisual(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node)
: scene_manager_(scene_manager), parent_scene_node_(parent_scene_node)
{
  scene_node_ = parent_scene_node_->createChildSceneNode();
  bbox_ = std::make_unique<Shape>(
    Shape::Type::Cube, scene_manager_,
    scene_node_->createChildSceneNode());
  covariance_ = std::make_unique<CovarianceVisual>(
    scene_manager_,
    scene_node_->createChildSceneNode());
}

DetectionVisual::~DetectionVisual()
{
  if (scene_node_) {
    parent_scene_node_->removeChild(scene_node_);
  }
}

rviz_rendering::Shape & DetectionVisual::bbox()
{
  return *bbox_;
}

rviz_rendering::CovarianceVisual & DetectionVisual::covariance()
{
  return *covariance_;
}

void DetectionVisual::update(const vision_msgs::msg::Detection3D & detection)
{
  const auto & result = detection.results.front();
  scene_node_->setPosition(rviz_common::pointMsgToOgre(result.pose.pose.position));
  scene_node_->setOrientation(rviz_common::quaternionMsgToOgre(result.pose.pose.orientation));

  bbox_->setPosition(rviz_common::pointMsgToOgre(detection.bbox.center.position));
  bbox_->setOrientation(rviz_common::quaternionMsgToOgre(detection.bbox.center.orientation));
  bbox_->setScale(rviz_common::vector3MsgToOgre(detection.bbox.size));

  covariance_->setPosition(bbox_->getPosition());
  covariance_->setOrientation(bbox_->getOrientation());
  covariance_->setCovariance(
    rviz_common::quaternionMsgToOgre(result.pose.pose.orientation), result.pose.covariance);
}

} // namespace vision_rviz_plugins
