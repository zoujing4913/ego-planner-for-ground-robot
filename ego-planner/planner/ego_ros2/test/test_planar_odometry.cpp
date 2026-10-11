#include <gtest/gtest.h>
#include <ego_ros2/node_utils.hpp>
#include <ego_ros2/planar_odometry.hpp>

class RosTest : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(RosTest, LocalParametersUseRos2NamesAndOverrides)
{
  auto options = rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("MPC.v_max", 0.7)});
  auto node = std::make_shared<rclcpp::Node>("parameter_test", options);
  double value;
  ego_ros2::parameter(node, "MPC/v_max", value, 1.0);
  EXPECT_DOUBLE_EQ(value, 0.7);
  ego_ros2::use_clock(node);
  EXPECT_EQ(ego_ros2::now().get_clock_type(), node->get_clock()->get_clock_type());
}

TEST_F(RosTest, BodyVelocityAndPoseTransformIntoThePlanningFrame)
{
  auto node = std::make_shared<rclcpp::Node>("odometry_test");
  tf2_ros::Buffer buffer(node->get_clock());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "odom";
  transform.transform.translation.x = 10.0;
  transform.transform.translation.y = -3.0;
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, std::acos(-1.0) / 2.0);
  transform.transform.rotation = tf2::toMsg(rotation);
  ASSERT_TRUE(buffer.setTransform(transform, "test", true));
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.position.x = 1.0;
  odometry.pose.pose.position.y = 2.0;
  odometry.pose.pose.position.z = 0.4;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 1.0;
  const auto state = ego_ros2::planar_odometry(odometry, buffer, "map");
  EXPECT_EQ(state.pose.header.frame_id, "map");
  EXPECT_NEAR(state.pose.pose.position.x, 8.0, 1e-9);
  EXPECT_NEAR(state.pose.pose.position.y, -2.0, 1e-9);
  EXPECT_DOUBLE_EQ(state.pose.pose.position.z, 0.0);
  EXPECT_NEAR(state.velocity.x(), 0.0, 1e-9);
  EXPECT_NEAR(state.velocity.y(), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(state.velocity.z(), 0.0);
}

TEST_F(RosTest, InvalidQuaternionIsRejected)
{
  auto node = std::make_shared<rclcpp::Node>("invalid_odometry_test");
  tf2_ros::Buffer buffer(node->get_clock());
  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "odom";
  odometry.pose.pose.orientation.w = 0.0;
  EXPECT_THROW(ego_ros2::planar_odometry(odometry, buffer, "odom"), std::invalid_argument);
}
