#include "ellipsoid_array_display.h"

namespace decomp_rviz_plugins {

EllipsoidArrayDisplay::EllipsoidArrayDisplay() {
  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(204, 51, 204),
                                            "Color of ellipsoids.",
                                            this, SLOT(updateColorAndAlpha()));
  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 0.5, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));

}

void EllipsoidArrayDisplay::onInitialize() {
  MFDClass::onInitialize();
}

EllipsoidArrayDisplay::~EllipsoidArrayDisplay() {}

void EllipsoidArrayDisplay::reset() {
  MFDClass::reset();
  visual_ = nullptr;
}

void EllipsoidArrayDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  if (visual_)
    visual_->setColor(color.r, color.g, color.b, alpha);
}

void EllipsoidArrayDisplay::processMessage(decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    RVIZ_COMMON_LOG_DEBUG("Error transforming from frame '" + msg->header.frame_id + "' to frame '" + fixed_frame_.toStdString() + "'");
    return;
  }

  std::shared_ptr<EllipsoidArrayVisual> visual;
  visual.reset(new EllipsoidArrayVisual(context_->getSceneManager(), scene_node_));

  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);

  visual_ = visual;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(decomp_rviz_plugins::EllipsoidArrayDisplay, rviz_common::Display)
