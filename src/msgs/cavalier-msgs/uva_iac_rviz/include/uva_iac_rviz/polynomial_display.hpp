/*
 * Copyright (c) 2020 Adam Gotlib (WUT Driverless)
 * For internal use of Indy Autonomous Challenge participants
 * DO NOT REDISTRIBUTE. See README.md for details
 */

#ifndef UVA_IAC_RVIZ_POLYNOMIAL_DISPLAY_HPP
#define UVA_IAC_RVIZ_POLYNOMIAL_DISPLAY_HPP

#ifndef Q_MOC_RUN

#include <rviz_common/message_filter_display.hpp>
#include <uva_iac_msgs/msg/polynomial.hpp>

#endif

namespace rviz_common {

namespace properties {

class ColorProperty;
class FloatProperty;
class IntProperty;

}  // namespace properties
}  // namespace rviz_common

namespace uva_iac_rviz {

class PolynomialVisual;

class PolynomialDisplay : public rviz_common::MessageFilterDisplay<uva_iac_msgs::msg::Polynomial> {
  Q_OBJECT
 public:
  PolynomialDisplay();
  ~PolynomialDisplay() override;

  void onInitialize() override;
  void reset() override;

 private Q_SLOTS:
  void updateColorAndAlpha();
  void updateNumPoints();

 private:
  void processMessage(const uva_iac_msgs::msg::Polynomial::ConstSharedPtr msg) override;

  PolynomialVisual* visual_;

  // User-editable property variables.
  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::IntProperty* num_points_property_;
};

}  // namespace uva_iac_rviz

#endif  // UVA_IAC_RVIZ_POLYNOMIAL_DISPLAY_HPP
