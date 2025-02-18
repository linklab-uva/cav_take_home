#pragma once

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <perception_utils/point_types.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <utils/kdtree.hpp>
#include <utils/nanoflann.hpp>
#include <utils/path_types.hpp>
#include <uva_iac_msgs/msg/batch_track.hpp>
#include <uva_iac_msgs/msg/batch_track_prediction.hpp>
#include <uva_iac_msgs/msg/track.hpp>
#include <uva_iac_msgs/msg/track_prediction.hpp>

class TrajectoryPropagator : public rclcpp::Node {
#define MAX_NUM_OF_OPPONENTS 10
  // #define USING_OLD_BAG
  using Position = utils::Position;
  const double DELTA_X = 0.1;
  const double DELTA_Y = 0.1;

 public:
  TrajectoryPropagator(const rclcpp::NodeOptions& options);

 private:
  // Track name, prediction horizon, process batch track, and maximum prediction distance
  std::string track_name_;
  double prediction_horizon_;
  double max_prediction_dist_;

  // Use the Position, PositionCloud and position_kdtree from utils
  std::vector<Position> track_refline_;
  utils::PositionCloud track_cloud_;
  std::unique_ptr<utils::position_kdtree> track_refline_tree_;

  std::vector<Position> pits_refline_;
  utils::PositionCloud pits_cloud_;
  std::unique_ptr<utils::position_kdtree> pits_refline_tree_;
  pcl::PointCloud<uva_iac::perception::PointTimeType>::Ptr output_cloud;

  // Subscribers and Publishers
  rclcpp::Subscription<uva_iac_msgs::msg::BatchTrack>::SharedPtr batch_track_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_name_sub_;
  rclcpp::Publisher<uva_iac_msgs::msg::BatchTrackPrediction>::SharedPtr track_prediction_pub_;

  // Message storage
  std::vector<uva_iac_msgs::msg::TrackPrediction> track_prediction_objs_;
  std::vector<sensor_msgs::msg::PointCloud2> prediction_trajectories_;
  uva_iac_msgs::msg::BatchTrackPrediction batch_track_prediction_;

  /**
   * @brief Add velocity for this track to queue and then take moving average of last velocities
   * @param ts[in]: timestamp of track
   */
  double get_velocity(const uva_iac_msgs::msg::Track& track, double ts);

  /**
   * @brief Add acceleration for this track to queue and then take moving average of last accels
   */
  double get_acceleration(const uva_iac_msgs::msg::Track& track);

  /**
   * @brief Compute average of history
   */
  static double calculate_average(const std::deque<double>& values) {
    if (values.empty()) {
      return 0.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  }

  /**
   * @brief Removes really old tracks from our history
   */
  template <typename T>
  static void remove_old_track_ids(std::unordered_map<uint32_t, std::deque<T>>& history_map, uint32_t track_id) {
    for (auto it = history_map.begin(); it != history_map.end();) {
      if (track_id >= it->first + MAX_TRACKS_IN_MEMORY) {
        it = history_map.erase(it);
        continue;
      }
      ++it;
    }
  }

  bool use_pit_refline{false};

  // Callback functions
  void batch_track_callback(const uva_iac_msgs::msg::BatchTrack::SharedPtr msg);
  void path_name_callback(const std_msgs::msg::String::SharedPtr msg);

  sensor_msgs::msg::PointCloud2 process_pose_with_covariance(const uva_iac_msgs::msg::Track& opponent_pose,
                                                             const std_msgs::msg::Header& header,
                                                             sensor_msgs::msg::PointCloud2& pointcloud_out, double ts);

  std::vector<Position> load_reference_line(const std::string& line);

  static constexpr size_t MAX_HISTORY{10U};
  static constexpr size_t MAX_TRACKS_IN_MEMORY{12U};

  static constexpr size_t MAX_HISTORY_FOR_FIT{50U};

  std::unordered_map<uint32_t, std::deque<double>> velocity_history_;
  std::unordered_map<uint32_t, std::deque<std::pair<double, double>>> velocity_history_for_fit_;

#ifdef USING_OLD_BAG
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opponent_odom_sub;
  void opponent_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
#endif
};
