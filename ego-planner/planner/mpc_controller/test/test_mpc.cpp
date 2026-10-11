#include <gtest/gtest.h>
#include <MPC.hpp>

TEST(Mpc, StraightReferenceProducesFiniteBoundedDifferentialDriveControls)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("mpc_test");
  MPC_controller controller;
  controller.MPC_init(node);
  std::vector<Eigen::Vector3d> states;
  std::vector<Eigen::Vector2d> controls;
  for (int i = 0; i < 15; ++i) {
    states.emplace_back(0.03 * i * 0.5, 0.0, 0.0);
    controls.emplace_back(0.5, 0.0);
  }
  const auto result = controller.MPC_Solve_qp(Eigen::Vector3d::Zero(), states, controls, 15);
  ASSERT_EQ(result.rows(), 2);
  ASSERT_EQ(result.cols(), 15);
  ASSERT_TRUE(result.allFinite());
  EXPECT_GT(result(0, 0), 0.0);
  for (int i = 0; i < result.cols(); ++i) {
    EXPECT_GE(result(0, i), -1e-8);
    EXPECT_LE(result(0, i), 1.0 + 1e-8);
    EXPECT_NEAR(result(1, i), 0.0, 1e-8);
  }
  rclcpp::shutdown();
}
