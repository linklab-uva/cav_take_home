/*
 * Copyright (c) 2020 Adam Gotlib (WUT Driverless)
 * For internal use of Indy Autonomous Challenge participants
 * DO NOT REDISTRIBUTE. See README.md for details
 */

#include "uva_iac_rviz/polynomial_visual.hpp"

#include <armadillo>

#include <math.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "uva_iac_rviz/objects/flat_color_line.hpp"

namespace uva_iac_rviz {

PolynomialVisual::PolynomialVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

PolynomialVisual::~PolynomialVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void PolynomialVisual::setMessage(const uva_iac_msgs::msg::Polynomial::ConstSharedPtr& msg) {
  line_segments_.clear();
  if ((msg->xmin == 0.0) && (msg->xmax == 0.0)) {
    return;
  } 
  if (msg->xmin >= msg->xmax) {
    return;
  }
  double start_x = msg->xmin;
  double end_x = msg->xmax;

  arma::vec xvec = arma::linspace(start_x, end_x, (arma::uword)this->num_points_);
  arma::vec P(msg->polynomial_coefficients);
  arma::vec yvec = arma::polyval(P, xvec);

  Ogre::Vector3 point;
  Ogre::Vector3 prev_point;

  for (unsigned int i = 0; i < this->num_points_; i++) {
    double x = xvec[i];
    double y = yvec[i];
    point = Ogre::Vector3((float)x, (float)y, 0.0);
    if (i > 0) {
      auto line_segment = std::make_shared<objects::FlatColorLine>(scene_manager_, frame_node_);

      line_segment->setPoints(prev_point, point);
      line_segment->setColor(color_.r, color_.g, color_.b, alpha_);

      line_segments_.push_back(line_segment);
    }
    prev_point = point;
  }
}

void PolynomialVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void PolynomialVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void PolynomialVisual::setColorAndAlpha(Ogre::ColourValue color, float alpha) {
  color_ = color;
  alpha_ = alpha;
}

void PolynomialVisual::setNumPoints(unsigned int num_points) {
  num_points_ = num_points;
}

}  // namespace uva_iac_rviz
