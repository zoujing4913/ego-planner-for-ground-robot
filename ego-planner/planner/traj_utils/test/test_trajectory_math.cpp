#include <gtest/gtest.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/polynomial_traj.h>

TEST(TrajectoryMath, UniformBsplinePreservesPlanarPositionAndDerivatives)
{
  Eigen::MatrixXd points(3, 8);
  for (int i = 0; i < 8; ++i) points.col(i) << i, 0.1 * i, 0.0;
  ego_planner::UniformBspline trajectory(points, 3, 0.2);
  const auto position = trajectory.evaluateDeBoorT(0.5);
  const auto velocity = trajectory.getDerivative().evaluateDeBoorT(0.5);
  EXPECT_NEAR(position.x(), 3.5, 1e-10);
  EXPECT_NEAR(position.y(), 0.35, 1e-10);
  EXPECT_DOUBLE_EQ(position.z(), 0.0);
  EXPECT_NEAR(velocity.x(), 5.0, 1e-10);
  EXPECT_NEAR(velocity.y(), 0.5, 1e-10);
  EXPECT_NEAR(trajectory.getDerivative().getDerivative().evaluateDeBoorT(0.5).norm(), 0.0, 1e-10);
}

TEST(TrajectoryMath, CubicPolynomialJerkIntegralIsUnchanged)
{
  PolynomialTraj trajectory;
  trajectory.addSegment({1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, 2.0);
  trajectory.init();
  EXPECT_NEAR(trajectory.evaluate(1.0).x(), 1.0, 1e-10);
  EXPECT_NEAR(trajectory.evaluateVel(1.0).x(), 3.0, 1e-10);
  EXPECT_NEAR(trajectory.evaluateAcc(1.0).x(), 6.0, 1e-10);
  EXPECT_NEAR(trajectory.getJerk(), 72.0, 1e-10);
}
