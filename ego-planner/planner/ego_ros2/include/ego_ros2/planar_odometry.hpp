#pragma once

#include <cmath>
#include <stdexcept>
#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace ego_ros2
{
struct PlanarOdometry
{
  geometry_msgs::msg::PoseStamped pose;
  Eigen::Vector3d velocity;
  double yaw;
};

inline PlanarOdometry planar_odometry(const nav_msgs::msg::Odometry &message,
                                      tf2_ros::Buffer &buffer, const std::string &frame)
{
  if (message.header.frame_id.empty()) throw std::invalid_argument("Odometry frame is empty");
  PlanarOdometry result;
  result.pose.header = message.header;
  result.pose.pose = message.pose.pose;
  if (message.header.frame_id != frame) result.pose = buffer.transform(result.pose, frame);
  tf2::Quaternion quaternion;
  tf2::fromMsg(result.pose.pose.orientation, quaternion);
  if (!std::isfinite(quaternion.length2()) || quaternion.length2() < 1e-12 ||
      !std::isfinite(result.pose.pose.position.x) || !std::isfinite(result.pose.pose.position.y) ||
      !std::isfinite(message.twist.twist.linear.x) || !std::isfinite(message.twist.twist.linear.y)) {
    throw std::invalid_argument("Odometry contains invalid position, velocity, or quaternion");
  }
  quaternion.normalize();
  double roll, pitch;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, result.yaw);
  quaternion.setRPY(0.0, 0.0, result.yaw);
  result.pose.pose.orientation = tf2::toMsg(quaternion);
  result.pose.pose.position.z = 0.0;
  result.pose.header.frame_id = frame;
  // nav_msgs/Odometry specifies twist in child_frame_id, not the world frame.
  const auto &velocity = message.twist.twist.linear;
  result.velocity << std::cos(result.yaw) * velocity.x - std::sin(result.yaw) * velocity.y,
                     std::sin(result.yaw) * velocity.x + std::cos(result.yaw) * velocity.y, 0.0;
  return result;
}
}  // namespace ego_ros2
