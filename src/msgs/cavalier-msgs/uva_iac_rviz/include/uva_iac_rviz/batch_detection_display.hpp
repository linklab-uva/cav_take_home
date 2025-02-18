#pragma once

#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <uva_iac_msgs/msg/batch_detection.hpp>

namespace uva_iac_rviz {

class BatchDetectionDisplay : public rviz_common::MessageFilterDisplay<uva_iac_msgs::msg::BatchDetection> {
  Q_OBJECT

 public:
  BatchDetectionDisplay() = default;

 protected:
  void onInitialize();
  void reset();
  void processMessage(uva_iac_msgs::msg::BatchDetection::ConstSharedPtr msg);

 private:
  /// Store arrows for visualizing all detections as orange arrows
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> detection_arrows_;

  /// MARK: Drawing constants
  const double shaft_length = 0.3;                 ///< Shaft length in meters
  const double shaft_diameter = 0.05;              ///< Shaft diameter in meters
  const double head_length = 0.2;                  ///< Head length in meters
  const double head_diameter = 0.1;                ///< Head diameter in meters
  std::array<float, 4> color{1.0, 0.0, 0.0, 1.0};  ///< Arrow color
};

}  // namespace uva_iac_rviz
