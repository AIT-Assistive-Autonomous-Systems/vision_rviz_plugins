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
#include <image_geometry/pinhole_camera_model.h>

namespace vision_rviz_plugins
{

class CameraInfoVisual
{
private:
  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode * parent_scene_node_;
  Ogre::SceneNode * scene_node_;

  Ogre::ManualObject * object_;
  Ogre::MaterialPtr wireframe_material;
  Ogre::MaterialPtr fill_material_;

  static std::atomic_uint64_t unique_ids_;

  double x_scale_, y_scale_;
  double far_distance_;

  void addFrustumPositions(Ogre::ManualObject * object);
  void generateWireframe(Ogre::ManualObject * object);
  void generateFill(Ogre::ManualObject * object);
  void generateObjects();

public:
  CameraInfoVisual(
    Ogre::SceneManager *, Ogre::SceneNode * parent_scene_node);

  virtual ~CameraInfoVisual();

  void setColor(Ogre::ColourValue);

  void setFarDistance(double d);

  void update(const image_geometry::PinholeCameraModel &);
};

} // namespace vision_rviz_plugins
