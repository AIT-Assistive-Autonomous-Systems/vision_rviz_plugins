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
