#include <ego_ros2/node_utils.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("ego_planner_node");
  ego_ros2::use_clock(nh);

  EGOReplanFSM rebo_replan;

  rebo_replan.init(nh);


  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0)));
  rclcpp::spin(nh);
  rclcpp::shutdown();

  return 0;
}
