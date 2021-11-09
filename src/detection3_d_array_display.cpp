#include "detection3_d_array_display.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/covariance_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace vision_rviz_plugins
{

using vision_msgs::msg::Detection3DArray;
using rviz_common::properties::StatusProperty;
using rviz_rendering::Shape;
using rviz_rendering::CovarianceVisual;

Detection3DArrayDisplay::Detection3DArrayDisplay()
{
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the bounding boxes.",
    this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the bounding boxes.",
    this, SLOT(updateColorAndAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  covariance_property_ = new rviz_common::properties::CovarianceProperty(
    "Covariance", true, "Whether or not the covariances of the messages should be shown.",
    this, SLOT(updateCovariance()));
}

Detection3DArrayDisplay::~Detection3DArrayDisplay() = default;

void Detection3DArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();
  detection_visuals_.clear();
}

void Detection3DArrayDisplay::onEnable()
{
  MFDClass::onEnable();
  // TODO visibility
}

void Detection3DArrayDisplay::onDisable()
{
  MFDClass::onDisable();
  // TODO visibility
}

void Detection3DArrayDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();
  for (auto & visual : detection_visuals_) {
    visual.bbox().setColor(color);
  }
}

void Detection3DArrayDisplay::updateCovariance()
{
  for (auto & visual : detection_visuals_) {
    visual.covariance().updateUserData(covariance_property_->getUserData());
  }
}

void Detection3DArrayDisplay::processMessage(Detection3DArray::ConstSharedPtr message)
{
  for (const auto & detection : message->detections) {
    if (!rviz_common::validateFloats(detection.bbox.center)) {
      setStatus(
        StatusProperty::Error, "Topic",
        "Message contained invalid floating point values (nans or infs) in bounding box center");
      return;
    }
    if (!rviz_common::validateFloats(detection.bbox.size)) {
      setStatus(
        StatusProperty::Error, "Topic",
        "Message contained invalid floating point values (nans or infs) in bounding box size");
      return;
    }
    // TODO(ZeilingerM) allow display of multiple results
    if (detection.results.size() != 1) {
      setStatus(StatusProperty::Error, "Detections", "Detections must have exactly one result");
      return;
    }
    for (const auto & result : detection.results) {
      if (!rviz_common::validateFloats(result.pose.pose)) {
        setStatus(
          StatusProperty::Error, "Topic",
          "Message contained invalid floating point values (nans or infs) in result pose");
        return;
      }
      if (!rviz_common::validateFloats(result.hypothesis.score)) {
        setStatus(
          StatusProperty::Error, "Topic",
          "Message contained invalid floating point values (nans or infs) in result hypothesis");
        return;
      }
    }
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(message->header, position, orientation)) {
    setMissingTransformToFixedFrame(message->header.frame_id);
    return;
  }
  setTransformOk();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  if (message->detections.size() < detection_visuals_.size()) {
    auto it = detection_visuals_.begin();
    std::advance(it, message->detections.size());
    detection_visuals_.erase(it, detection_visuals_.end());
  } else {
    while (detection_visuals_.size() < message->detections.size()) {
      detection_visuals_.emplace_back(scene_manager_, scene_node_);
    }
  }

  assert(detection_visuals_.size() == message->detections.size());
  auto di = message->detections.cbegin();
  for (auto vi = detection_visuals_.begin();
    vi != detection_visuals_.end(); ++vi, ++di)
  {
    vi->update(*di);
  }

  updateColorAndAlpha();
  updateCovariance();
}

void Detection3DArrayDisplay::reset()
{
  MFDClass::reset();
  detection_visuals_.clear();
}

}  // namespace vision_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(vision_rviz_plugins::Detection3DArrayDisplay, rviz_common::Display)
