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

const float fill_alpha_scale = 0.2;

CameraInfoVisual::CameraInfoVisual(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node)
: scene_manager_(scene_manager), parent_scene_node_(parent_scene_node), x_scale_(0.0),
  y_scale_(0.0),
  far_distance_(0.0)
{
  auto inc_id = unique_ids_.fetch_add(1);

  scene_node_ = parent_scene_node_->createChildSceneNode();

  auto wireframe_id = "graph_rviz_plugins_camera_info/wireframe" + std::to_string(inc_id);
  object_ = scene_manager_->createManualObject(wireframe_id);
  scene_node_->attachObject(object_);
  wireframe_material = rviz_rendering::MaterialManager::createMaterialWithLighting(wireframe_id);

  auto fill_id = "graph_rviz_plugins_camera_info/fill" + std::to_string(inc_id);
  fill_material_ = rviz_rendering::MaterialManager::createMaterialWithLighting(fill_id);
  fill_material_->setCullingMode(Ogre::CULL_NONE);
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
  rviz_rendering::MaterialManager::enableAlphaBlending(wireframe_material, color.a);
  wireframe_material->setAmbient(color * 0.5f);
  wireframe_material->setDiffuse(color);
  wireframe_material->setSelfIllumination(color);

  color.a *= fill_alpha_scale;
  rviz_rendering::MaterialManager::enableAlphaBlending(fill_material_, color.a);
  fill_material_->setAmbient(color * 0.5f);
  fill_material_->setDiffuse(color);
  fill_material_->setSelfIllumination(color);
}

void CameraInfoVisual::setFarDistance(double d)
{
  if (far_distance_ != d) {
    far_distance_ = d;
    generateObjects();
  }
}

void CameraInfoVisual::addFrustumPositions(Ogre::ManualObject * object)
{
  // near plane (single projection center)
  object->position(0, 0, 0);

  // far plane
  auto far_width = far_distance_ * x_scale_;
  auto far_height = far_distance_ * y_scale_;

  // top left
  object->position(-far_width / 2.0, -far_height / 2.0, far_distance_);
  // top right
  object->position(+far_width / 2.0, -far_height / 2.0, far_distance_);
  // bottom right
  object->position(+far_width / 2.0, +far_height / 2.0, far_distance_);
  // bottom left
  object->position(-far_width / 2.0, +far_height / 2.0, far_distance_);
}

void CameraInfoVisual::generateWireframe(Ogre::ManualObject * object)
{
  object->begin(wireframe_material->getName(), Ogre::RenderOperation::OT_LINE_LIST, "rviz_rendering");
  addFrustumPositions(object);
  // viewing direction lines
  for (size_t i = 0; i < 4; i++) {
    object->index(0);
    object->index(1 + i);
  }
  // far plane rectangle
  for (size_t i = 0; i < 4; i++) {
    object->index(1 + i);
    object->index(1 + ((i + 1) % 4));
  }
  object->end();
}

void CameraInfoVisual::generateFill(Ogre::ManualObject * object)
{
  object->begin(
    fill_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  addFrustumPositions(object);
  for (size_t i = 0; i < 4; i++) {
    object->index(0);
    object->index(1 + ((i + 1) % 4));
    object->index(1 + ((i) % 4));
  }
  object->end();
}

void CameraInfoVisual::generateObjects()
{
  object_->clear();
  if (x_scale_ == 0.0 || y_scale_ == 0.0 || far_distance_ == 0.0) {
    return;
  }
  generateFill(object_);
  generateWireframe(object_);
}

void CameraInfoVisual::update(const image_geometry::PinholeCameraModel & camera)
{
  assert(camera.fx() > 0.0 && camera.fy() > 0.0);

  auto new_x_scale = camera.reducedResolution().width / camera.fx();
  auto new_y_scale = camera.reducedResolution().height / camera.fy();

  if (x_scale_ != new_x_scale || y_scale_ != new_y_scale) {
    x_scale_ = new_x_scale;
    y_scale_ = new_y_scale;
    generateObjects();
  }
}

} // namespace vision_rviz_plugins
