/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mesh_shape.hpp"

#include <cstdint>
#include <string>

#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreVector3.h>

#include "rviz_rendering/material_manager.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace vision_rviz_plugins
{

using rviz_rendering::Shape;
using rviz_rendering::MaterialManager;

MeshShape::MeshShape(
  std::string mesh,
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_node)
: scene_manager_(scene_manager), parent_node_(parent_node)
{
  static uint32_t count = 0;
  std::string entity_name = "MeshShape" + std::to_string(count++);
  entity_ = scene_manager_->createEntity(
    entity_name, mesh, "rviz_rendering");
  assert(entity_);

  if (!parent_node_) {
    parent_node_ = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node_->createChildSceneNode();
  offset_node_ = scene_node_->createChildSceneNode();
  if (entity_) {
    offset_node_->attachObject(entity_);
  }

  material_name_ = entity_name + "Material";
  material_ = MaterialManager::createMaterialWithLighting(material_name_);
  material_->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);

  if (entity_) {
    entity_->setMaterialName(material_name_);
  }
}

MeshShape::~MeshShape()
{
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroySceneNode(offset_node_);

  if (entity_) {
    scene_manager_->destroyEntity(entity_);
  }
  material_->unload();
  Ogre::MaterialManager::getSingleton().remove(material_->getName(), "rviz_rendering");
}

void MeshShape::setColor(const Ogre::ColourValue & c)
{
  material_->getTechnique(0)->setAmbient(c * 0.5);
  material_->getTechnique(0)->setDiffuse(c);
  MaterialManager::enableAlphaBlending(material_, c.a);
}

}  // namespace vision_rviz_plugins
