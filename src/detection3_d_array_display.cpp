#include "detection3_d_array_display.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/covariance_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/msg_conversions.hpp"

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
  detections_.clear();
  covariances_.clear();
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
  for (auto & bbox : detections_) {
    bbox.setColor(color);
  }
  context_->queueRender();
}

void Detection3DArrayDisplay::updateCovariance()
{
  for (auto & covariance : covariances_) {
    covariance.updateUserData(covariance_property_->getUserData());
  }
  context_->queueRender();
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

  if (message->detections.size() < detections_.size()) {
    detections_.erase(detections_.begin() + message->detections.size(), detections_.end());
  } else {
    while (detections_.size() < message->detections.size()) {
      detections_.emplace_back(Shape::Type::Cube, scene_manager_, scene_node_);
    }
  }
  covariances_.clear();

  for (size_t i = 0; i < detections_.size(); i++) {
    const auto & detection = message->detections[i];
    // TODO(ZeilingerM) allow display of multiple results
    if (detection.results.size() != 1) {
      setStatus(StatusProperty::Warn, "Detections", "Detections must have exactly one result");
    }
    const auto & result = detection.results.front();
    auto & bbox = detections_[i];
    // TODO(ZeilingerM) display bbox offset
    bbox.setPosition(rviz_common::pointMsgToOgre(result.pose.pose.position));
    bbox.setOrientation(rviz_common::quaternionMsgToOgre(result.pose.pose.orientation));
    bbox.setScale(rviz_common::vector3MsgToOgre(detection.bbox.size));

    auto & covariance = covariances_.emplace_back(scene_manager_, scene_node_);
    covariance.setPosition(bbox.getPosition());
    covariance.setOrientation(bbox.getOrientation());
    covariance.setCovariance(
      rviz_common::quaternionMsgToOgre(result.pose.pose.orientation), result.pose.covariance);
    covariance.updateUserData(covariance_property_->getUserData());
  }

  updateColorAndAlpha();
}

void Detection3DArrayDisplay::reset()
{
  MFDClass::reset();
  detections_.clear();
  covariances_.clear();
}

}  // namespace vision_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(vision_rviz_plugins::Detection3DArrayDisplay, rviz_common::Display)
