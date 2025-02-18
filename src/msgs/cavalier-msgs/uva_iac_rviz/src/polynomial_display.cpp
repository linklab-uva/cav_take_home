/*
 * Copyright (c) 2020 Adam Gotlib (WUT Driverless)
 * For internal use of Indy Autonomous Challenge participants
 * DO NOT REDISTRIBUTE. See README.md for details
 */

#include "uva_iac_rviz/polynomial_display.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>

#include "uva_iac_rviz/polynomial_visual.hpp"

namespace uva_iac_rviz {

PolynomialDisplay::PolynomialDisplay() : visual_(nullptr) {
  color_property_ = new rviz_common::properties::ColorProperty(
      "Color", QColor(255, 255, 255), "Color to draw the road lines with.", this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this, SLOT(updateColorAndAlpha()));

  num_points_property_ = new rviz_common::properties::IntProperty(
      "Number of Points", 100, "How many points to render on the polynomial.", this, SLOT(updateNumPoints()));
  num_points_property_->setMin(2);
  num_points_property_->setMax(10000);
}

void PolynomialDisplay::onInitialize() {
  MFDClass::onInitialize();
}

PolynomialDisplay::~PolynomialDisplay() {}

// Clear the visual by deleting its object.
void PolynomialDisplay::reset() {
  MFDClass::reset();
  if (visual_)
    delete visual_;
}

// Set the current color and alpha values for the visual.
void PolynomialDisplay::updateColorAndAlpha() {
  if (!visual_)
    return;

  Ogre::ColourValue color = color_property_->getOgreColor();
  float alpha = alpha_property_->getFloat();

  visual_->setColorAndAlpha(color, alpha);
}

// Set the current rendering range for the visual.
void PolynomialDisplay::updateNumPoints() {
  if (!visual_)
    return;

  int num_points = num_points_property_->getInt();

  visual_->setNumPoints((unsigned int)num_points);
}

// This is our callback to handle an incoming message.
void PolynomialDisplay::processMessage(const uva_iac_msgs::msg::Polynomial::ConstSharedPtr msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, position, orientation)) {
    // TODO Raise an error
    return;
  }

  // Create and configure new visual if it doesn't exist yet.
  if (!visual_) {
    visual_ = new PolynomialVisual(context_->getSceneManager(), scene_node_);

    updateColorAndAlpha();
    updateNumPoints();
  }

  // Now set or update the contents of the visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // namespace uva_iac_rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uva_iac_rviz::PolynomialDisplay, rviz_common::Display)
