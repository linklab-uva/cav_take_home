#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/objects/arrow.hpp>
#include <uva_iac_rviz/batch_tracks_display.hpp>

namespace uva_iac_rviz {

void BatchTracksDisplay::onInitialize() {
  MFDClass::onInitialize();
}

void BatchTracksDisplay::reset() {
  MFDClass::reset();
  track_arrows_.clear();
}

void BatchTracksDisplay::processMessage(const uva_iac_msgs::msg::BatchTrack::ConstSharedPtr msg) {
  RVIZ_COMMON_LOG_INFO_STREAM("Received BatchTracks message with " << msg->tracks.size() << " tracks.");

  track_arrows_.clear();
  track_arrows_.reserve(msg->tracks.size());

  for (const auto& track : msg->tracks) {
    Ogre::Vector3 scene_position;
    Ogre::Quaternion scene_orientation;

    if (!context_->getFrameManager()->transform(msg->header, track.pose.pose, scene_position, scene_orientation)) {
      RVIZ_COMMON_LOG_WARNING_STREAM("Failed to transform track with ID: " << track.track_id);
      continue;
    }

    Ogre::Vector3 linear_velocity(static_cast<float>(track.twist.twist.linear.x),
                                  static_cast<float>(track.twist.twist.linear.y),
                                  static_cast<float>(track.twist.twist.linear.z));

    auto arrow = std::make_unique<rviz_rendering::Arrow>(context_->getSceneManager(), scene_node_, shaft_length * 100,
                                                         shaft_diameter * 100, head_length * 100, head_diameter * 100);
    arrow->setPosition(scene_position);
    arrow->setDirection(linear_velocity.normalisedCopy());
    arrow->setColor(color[0], color[1], color[2], color[3]);
    track_arrows_.push_back(std::move(arrow));
  }
}

}  // namespace uva_iac_rviz

PLUGINLIB_EXPORT_CLASS(uva_iac_rviz::BatchTracksDisplay, rviz_common::Display)
