/*
 * Copyright (c) 2020 Adam Gotlib (WUT Driverless)
 * For internal use of Indy Autonomous Challenge participants
 * DO NOT REDISTRIBUTE. See README.md for details
 */

#pragma once

#include <OgreVector3.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <uva_iac_msgs/msg/polynomial.hpp>

namespace uva_iac_rviz {

namespace objects {
class FlatColorLine;
}

// Each instance of PolynomialVisual represents the visualization
// of a single uva_iac_msgs/Polynomial message
class PolynomialVisual {
 public:
  PolynomialVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~PolynomialVisual();

  // Configure the visual to show the data in the message.
  void setMessage(const uva_iac_msgs::msg::Polynomial::ConstSharedPtr& msg);

  // Set the pose of the coordinate frame the message refers to.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the RoadMarkings message.
  void setColorAndAlpha(Ogre::ColourValue color, float alpha);

  // Set rendering range for the lines.
  void setNumPoints(unsigned int num_points);

 private:
  unsigned int num_points_;

  Ogre::ColourValue color_;
  float alpha_;

  // Objects representing the actual lines to be displayed.
  std::vector<std::shared_ptr<objects::FlatColorLine>> line_segments_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};

}  // namespace uva_iac_rviz
