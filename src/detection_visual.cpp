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

#include "detection_visual.hpp"
#include "rviz_common/msg_conversions.hpp"
#include <rcpputils/asserts.hpp>
#include <string>
#include <utility>
#include <memory>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace vision_rviz_plugins
{
using rviz_rendering::Axes;
using rviz_rendering::MovableText;
using rviz_rendering::Shape;
using rviz_rendering::CovarianceVisual;

const float ID_TEXT_OFFSET = 0.2f;

DetectionVisual::DetectionVisual(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node)
: scene_manager_(scene_manager), parent_scene_node_(parent_scene_node)
{
  scene_node_ = parent_scene_node_->createChildSceneNode();
  bbox_node_ = scene_node_->createChildSceneNode();
  bbox_ = std::make_unique<Shape>(
    Shape::Type::Cube, scene_manager_,
    bbox_node_);
  axes_ = std::make_unique<Axes>(
    scene_manager_,
    scene_node_->createChildSceneNode());

  id_text_ = std::make_unique<MovableText>("", "Liberation Sans", 0.2f);
  id_text_->setTextAlignment(MovableText::H_CENTER, MovableText::V_CENTER);
  scene_node_->attachObject(id_text_.get());

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

void DetectionVisual::setAxes(double length, double radius)
{
  axes_->set(length, radius);
}

rviz_rendering::Shape & DetectionVisual::bbox()
{
  return *bbox_;
}

rviz_rendering::CovarianceVisual & DetectionVisual::covariance()
{
  return *covariance_;
}

void DetectionVisual::setColor(Ogre::ColourValue color)
{
  bbox_->setColor(color);
  if (mesh_) {
    mesh_->setColor(color);
  }
  color.a = 1.0; // force text alpha to opaque
  id_text_->setColor(color);
}

void DetectionVisual::setShowId(bool show)
{
  if (id_text_->getCaption().empty()) {
    show = false;
  }
  id_text_->setVisible(show);
}

void DetectionVisual::setMesh(const Ogre::MeshPtr & mesh)
{
  if (mesh) {
    mesh_ = std::make_unique<MeshShape>(mesh, scene_manager_, scene_node_->createChildSceneNode());
  } else {
    mesh_.reset();
  }
}

void DetectionVisual::update(
  const vision_msgs::msg::Detection3D & detection,
  Ogre::Vector3 height_axis)
{
  rcpputils::assert_true(!detection.results.empty(), "detection does not contain results");
  const auto & result = detection.results.front();

  scene_node_->setPosition(rviz_common::pointMsgToOgre(result.pose.pose.position));
  scene_node_->setOrientation(rviz_common::quaternionMsgToOgre(result.pose.pose.orientation));

  auto bbox_offset = rviz_common::pointMsgToOgre(detection.bbox.center.position);
  bbox_->setPosition(bbox_offset);
  bbox_->setOrientation(rviz_common::quaternionMsgToOgre(detection.bbox.center.orientation));

  auto bbox_scale = rviz_common::vector3MsgToOgre(detection.bbox.size);
  bbox_->setScale(bbox_scale);

  covariance_->setCovariance(
    rviz_common::quaternionMsgToOgre(result.pose.pose.orientation), result.pose.covariance);

  // project on local height axis (ROS z is OGRE x)
  auto bbox_scale_along_height = bbox_scale.dotProduct(Ogre::Vector3::UNIT_X) / 2.0;
  auto bbox_offset_along_height = bbox_offset.dotProduct(Ogre::Vector3::UNIT_X) / 2.0;
  id_text_->setLocalTranslation(
    height_axis *
    (bbox_scale_along_height + bbox_offset_along_height + ID_TEXT_OFFSET));

  if (detection.id.empty()) {
    id_text_->setCaption("");
    setShowId(false);
  } else {
    id_text_->setCaption("#" + detection.id);
  }

  // render bbox or mesh exclusively
  bbox_node_->setVisible(!mesh_);
}

} // namespace vision_rviz_plugins
