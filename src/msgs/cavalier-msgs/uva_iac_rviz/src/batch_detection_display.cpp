#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <uva_iac_rviz/batch_detection_display.hpp>

namespace uva_iac_rviz {

void BatchDetectionDisplay::onInitialize() {
  MFDClass::onInitialize();
}

void BatchDetectionDisplay::reset() {
  MFDClass::reset();
  detection_arrows_.clear();
}

void BatchDetectionDisplay::processMessage(const uva_iac_msgs::msg::BatchDetection::ConstSharedPtr msg) {
  RVIZ_COMMON_LOG_INFO_STREAM("Received BatchDetection message with " << msg->detections.size() << " detections.");

  detection_arrows_.clear();
  detection_arrows_.reserve(msg->detections.size());

  for (const auto& detection : msg->detections) {
    Ogre::Vector3 scene_position;
    Ogre::Quaternion scene_orientation;

    if (!context_->getFrameManager()->transform(detection.header, detection.pose.pose, scene_position,
                                                scene_orientation)) {
      RVIZ_COMMON_LOG_WARNING_STREAM("Failed to transform detection");
      continue;
    }

    Ogre::Vector3 linear_velocity(static_cast<float>(detection.twist.twist.linear.x),
                                  static_cast<float>(detection.twist.twist.linear.y),
                                  static_cast<float>(detection.twist.twist.linear.z));

    auto arrow = std::make_unique<rviz_rendering::Arrow>(context_->getSceneManager(), scene_node_, shaft_length * 100,
                                                         shaft_diameter * 100, head_length * 100, head_diameter * 100);
    arrow->setPosition(scene_position);
    arrow->setDirection(linear_velocity.normalisedCopy());
    arrow->setColor(color[0], color[1], color[2], color[3]);
    detection_arrows_.push_back(std::move(arrow));
  }
}

}  // namespace uva_iac_rviz

PLUGINLIB_EXPORT_CLASS(uva_iac_rviz::BatchDetectionDisplay, rviz_common::Display)
