#include <plan_env/grid_map.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

void GridMap::laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan)
{
  if (!md_.has_odom_ || scan->header.frame_id.empty() || scan->ranges.empty() ||
      !std::isfinite(scan->angle_min) || !std::isfinite(scan->angle_increment) || scan->angle_increment == 0.0 ||
      !std::isfinite(scan->range_min) || !std::isfinite(scan->range_max) ||
      scan->range_min < 0.0 || scan->range_max <= scan->range_min) return;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(mp_.frame_id_, scan->header.frame_id,
                                            rclcpp::Time(scan->header.stamp));
  } catch (const tf2::TransformException &error) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "LaserScan transform unavailable: %s", error.what());
    return;
  }
  tf2::Transform sensor_to_world;
  tf2::fromMsg(transform.transform, sensor_to_world);
  const auto origin = sensor_to_world.getOrigin();
  const double maximum = mp_.max_ray_length_ > 0.0 ?
      std::min<double>(scan->range_max, mp_.max_ray_length_) : scan->range_max;
  std::vector<Eigen::Vector2i> hits;
  bool observed = false;
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double measured = scan->ranges[i];
    if (std::isnan(measured) || measured < scan->range_min || measured < 0.0 ||
        (std::isfinite(measured) && measured > scan->range_max)) continue;
    observed = true;
    const bool hit = std::isfinite(measured) && measured < maximum;
    const double range = std::min(measured, maximum);
    const double angle = scan->angle_min + i * scan->angle_increment;
    const auto endpoint = sensor_to_world * tf2::Vector3(range * std::cos(angle), range * std::sin(angle), 0.0);
    const int steps = std::max(1, static_cast<int>(std::ceil(range / mp_.resolution_)));
    for (int step = 0; step < steps; ++step) {
      const double fraction = static_cast<double>(step) / steps;
      const Eigen::Vector2d point(origin.x() + fraction * (endpoint.x() - origin.x()),
                                  origin.y() + fraction * (endpoint.y() - origin.y()));
      if (!isInMap2d(point)) continue;
      Eigen::Vector2i index;
      posToIndex2d(point, index);
      laser_occupancy_buffer_[toAddress2d(index.x(), index.y())] = 0;
    }
    const Eigen::Vector2d point(endpoint.x(), endpoint.y());
    if (hit && isInMap2d(point)) {
      Eigen::Vector2i index;
      posToIndex2d(point, index);
      hits.push_back(index);
    }
  }
  if (!observed) return;
  for (const auto &index : hits) laser_occupancy_buffer_[toAddress2d(index.x(), index.y())] = 1;
  std::fill(md_.occupancy_buffer_inflate_2d_.begin(), md_.occupancy_buffer_inflate_2d_.end(), 0);
  const int radius = static_cast<int>(std::ceil(mp_.obstacles_inflation2d_ / mp_.resolution_));
  for (int x = 0; x < mp_.map_voxel_num_.x(); ++x) {
    for (int y = 0; y < mp_.map_voxel_num_.y(); ++y) {
      if (!laser_occupancy_buffer_[toAddress2d(x, y)]) continue;
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          const int ix = x + dx, iy = y + dy;
          if (ix >= 0 && iy >= 0 && ix < mp_.map_voxel_num_.x() && iy < mp_.map_voxel_num_.y())
            md_.occupancy_buffer_inflate_2d_[toAddress2d(ix, iy)] = 1;
        }
      }
    }
  }
  md_.has_cloud_ = true;
  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  publishMap2D();
}
