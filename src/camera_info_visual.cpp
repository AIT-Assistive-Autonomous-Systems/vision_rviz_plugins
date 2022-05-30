// Copyright 2022 Austrian Institute of Technology GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rviz_rendering/material_manager.hpp>
#include "camera_info_visual.hpp"
#include <utility>
#include <memory>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>

namespace vision_rviz_plugins
{
using Ogre::ColourValue;

std::atomic_uint64_t CameraInfoVisual::unique_ids_ = 0;

CameraInfoVisual::CameraInfoVisual(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node)
: scene_manager_(scene_manager), parent_scene_node_(parent_scene_node), x_scale_(0.0),
  y_scale_(0.0),
  far_distance_(0.0)
{
  auto inc_id = unique_ids_.fetch_add(1);
  auto id = "graph_rviz_plugins_camera_info/" + std::to_string(inc_id);
  object_ = scene_manager_->createManualObject(id);
  scene_node_ = parent_scene_node_->createChildSceneNode();
  scene_node_->attachObject(object_);

  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(id);
  material_->setLightingEnabled(true);
  material_->setReceiveShadows(false);
  setColor(ColourValue(1.0, 0.0, 0.0, 1.0));
}

CameraInfoVisual::~CameraInfoVisual()
{
  if (object_) {
    scene_node_->detachObject(object_);
    scene_manager_->destroyManualObject(object_);
  }
  if (scene_node_) {
    scene_manager_->destroySceneNode(scene_node_);
  }
}

void CameraInfoVisual::setColor(ColourValue color)
{
  rviz_rendering::MaterialManager::enableAlphaBlending(material_, color.a);
  material_->setAmbient(color * 0.5f);
  material_->setDiffuse(color);
  material_->setSelfIllumination(color);
}

void CameraInfoVisual::setFarDistance(double d)
{
  if (far_distance_ != d) {
    far_distance_ = d;
    generateMesh();
  }
}

void CameraInfoVisual::generateMesh()
{
  object_->clear();

  if (x_scale_ == 0.0 || y_scale_ == 0.0 || far_distance_ == 0.0) {
    return;
  }

  object_->begin(material_->getName(), Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");

  // near plane (single projection center)
  object_->position(0, 0, 0);

  // far plane
  auto far_width = far_distance_ * x_scale_;
  auto far_height = far_distance_ * y_scale_;

  // top left
  object_->position(-far_width / 2.0, -far_height / 2.0, far_distance_);
  // top right
  object_->position(+far_width / 2.0, -far_height / 2.0, far_distance_);
  // bottom right
  object_->position(+far_width / 2.0, +far_height / 2.0, far_distance_);
  // bottom left
  object_->position(-far_width / 2.0, +far_height / 2.0, far_distance_);

  // viewing directions
  for (size_t i = 0; i < 4; i++) {
    object_->index(0);
    object_->index(1 + i);
  }

  // far plane polygon
  for (size_t i = 0; i < 4; i++) {
    object_->index(1 + i);
    object_->index(1 + ((i + 1) % 4));
  }
  object_->end();
}

void CameraInfoVisual::update(const image_geometry::PinholeCameraModel & camera)
{
  assert(camera.fx() > 0.0 && camera.fy() > 0.0);

  auto new_x_scale = camera.reducedResolution().width / camera.fx();
  auto new_y_scale = camera.reducedResolution().height / camera.fy();

  if (x_scale_ != new_x_scale || y_scale_ != new_y_scale) {
    x_scale_ = new_x_scale;
    y_scale_ = new_y_scale;
    generateMesh();
  }
}

} // namespace vision_rviz_plugins
