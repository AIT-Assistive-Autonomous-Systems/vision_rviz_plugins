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

#include "camera_info_display.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/covariance_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace vision_rviz_plugins
{

using rviz_common::properties::StatusProperty;

CameraInfoDisplay::CameraInfoDisplay()
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the frustum.",
    this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the bounding boxes.",
    this, SLOT(updateColorAndAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  far_distance_property_ = new rviz_common::properties::FloatProperty(
    "Far Distance", 10.0, "Distance of far plane.",
    this, SLOT(updateFarDistance()));
  far_distance_property_->setMin(0);
}

CameraInfoDisplay::~CameraInfoDisplay() = default;

void CameraInfoDisplay::onInitialize()
{
  MFDClass::onInitialize();
  visual_.reset();
}

void CameraInfoDisplay::updateColorAndAlpha()
{
  if (visual_) {
    Ogre::ColourValue color = color_property_->getOgreColor();
    color.a = alpha_property_->getFloat();
    visual_->setColor(color);
    context_->queueRender();
  }
}

void CameraInfoDisplay::updateFarDistance()
{
  if (visual_) {
    visual_->setFarDistance(far_distance_property_->getFloat());
    context_->queueRender();
  }
}

void CameraInfoDisplay::processMessage(CameraInfo::ConstSharedPtr message)
{
  if (!rviz_common::validateFloats(message->k)) {
    setStatus(
      StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs) in camera matrix");
    return;
  }
  if (0.0 >= message->k[3 * 0 + 0]) {
    setStatus(
      StatusProperty::Error, "Topic",
      "fx is non-positive, can't determine view frustum");
    return;
  }
  if (0.0 >= message->k[3 * 1 + 1]) {
    setStatus(
      StatusProperty::Error, "Topic",
      "fy is non-positive, can't determine view frustum");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(message->header, position, orientation)) {
    setMissingTransformToFixedFrame(message->header.frame_id);
    return;
  }
  setTransformOk();

  if (!visual_) {
    visual_.reset(new CameraInfoVisual(scene_manager_, scene_node_));
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  updateColorAndAlpha();
  updateFarDistance();
  visual_->update(message);
}

void CameraInfoDisplay::reset()
{
  MFDClass::reset();
  visual_.reset();
}

}  // namespace vision_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(vision_rviz_plugins::CameraInfoDisplay, rviz_common::Display)
