/*
 * Copyright (c) 2020 Adam Gotlib (WUT Driverless)
 * For internal use of Indy Autonomous Challenge participants
 * DO NOT REDISTRIBUTE. See README.md for details
 */

#ifndef FLAT_COLOR_LINE_HPP
#define FLAT_COLOR_LINE_HPP

#include <rviz_rendering/material_manager.hpp>
#include <rviz_rendering/objects/line.hpp>

namespace uva_iac_rviz {

namespace objects {

class FlatColorLine : public rviz_rendering::Line {
 public:
  explicit FlatColorLine(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node = nullptr)
      : rviz_rendering::Line(manager, parent_node) {
    manual_object_material_->setDiffuse(0, 0, 0, 0);
    manual_object_material_->setAmbient(0, 0, 0);
    manual_object_material_->setSpecular(0, 0, 0, 0);
  }

  void setColor(const Ogre::ColourValue& c) override {
    manual_object_material_->setSelfIllumination(c);
    rviz_rendering::MaterialManager::enableAlphaBlending(manual_object_material_, c.a);
  }

  void setColor(float r, float g, float b, float a) override { setColor(Ogre::ColourValue(r, g, b, a)); }
};  // class FlatColorLine

}  // namespace objects

}  // namespace uva_iac_rviz
#endif  // FLAT_COLOR_LINE_HPP
