#!/usr/bin/env python3
"""Run with ground_robot.launch.py in an isolated ROS domain; no hardware."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from ego_planner.msg import Bspline
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class Robot(Node):
    def __init__(self):
        super().__init__('navigation_smoke')
        self.x = self.y = self.yaw = 0.0
        self.command = Twist()
        self.commands = []
        self.trajectories = []
        self.maps = []
        self.odom = self.create_publisher(Odometry, 'odom', 10)
        self.scan = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.goal = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.create_subscription(Twist, 'cmd_vel', self.control, 10)
        self.create_subscription(Bspline, '/planning/bspline', self.trajectories.append, 10)
        self.create_subscription(OccupancyGrid, '/map_2d', self.maps.append, 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child, x in [('map', 'odom', 10.0), ('base_link', 'laser', 0.15)]:
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id, tf.child_frame_id = parent, child
            tf.transform.translation.x = x
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        self.static_tf.sendTransform(transforms)
        self.closest = float('inf')
        self.tick = 0
        self.previous_step = time.monotonic()

    def control(self, command):
        assert all(math.isfinite(v) for v in [command.linear.x, command.angular.z])
        assert abs(command.linear.x) <= 1.00001 and abs(command.angular.z) <= 1.20001
        assert command.linear.y == command.linear.z == command.angular.x == command.angular.y == 0
        self.command = command
        self.commands.append(command)

    def step(self, scan=True, obstacle=True):
        self.tick += 1
        started = time.monotonic()
        dt = started - self.previous_step
        self.previous_step = started
        self.x += self.command.linear.x * math.cos(self.yaw) * dt
        self.y += self.command.linear.x * math.sin(self.yaw) * dt
        self.yaw += self.command.angular.z * dt
        self.closest = min(self.closest, math.hypot(self.x - 1.5, self.y))
        stamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        msg.twist.twist = self.command
        self.odom.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)
        if scan and self.tick % 5 == 0:
            lidar = LaserScan()
            lidar.header.stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=0.04)).to_msg()
            lidar.header.frame_id = 'laser'
            lidar.angle_min, lidar.angle_increment = -math.pi, math.pi/180
            lidar.angle_max = math.pi - lidar.angle_increment
            lidar.range_min, lidar.range_max = 0.05, 10.0
            sx = self.x + 0.15 * math.cos(self.yaw)
            sy = self.y + 0.15 * math.sin(self.yaw)
            for i in range(360):
                angle = self.yaw + lidar.angle_min + i * lidar.angle_increment
                dx, dy = math.cos(angle), math.sin(angle)
                ox, oy = sx - 1.5, sy
                b = ox*dx + oy*dy
                disc = b*b - (ox*ox + oy*oy - 0.2**2)
                distance = -b - math.sqrt(disc) if obstacle and disc >= 0 else float('inf')
                lidar.ranges.append(distance if distance >= 0.05 else float('inf'))
            self.scan.publish(lidar)
        end = started + 0.02
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.001)

    def run(self):
        ready_deadline = time.monotonic() + 10
        while time.monotonic() < ready_deadline:
            self.step()
            if self.tick > 100 and self.maps and any(v == 100 for v in self.maps[-1].data):
                break
        assert self.maps and any(v == 100 for v in self.maps[-1].data), 'obstacle map not ready before goal'
        grid = self.maps[-1]
        occupied = [(grid.info.origin.position.x + (i % grid.info.width + 0.5)*grid.info.resolution,
                     grid.info.origin.position.y + (i // grid.info.width + 0.5)*grid.info.resolution)
                    for i, value in enumerate(grid.data) if value == 100]
        print(f'Initial occupied bounds: {min(x for x,y in occupied)}, {max(x for x,y in occupied)}, {min(y for x,y in occupied)}, {max(y for x,y in occupied)}', flush=True)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.orientation.w = 13.0, 1.0
        self.goal.publish(goal)
        deadline = time.monotonic() + 40
        paused = False
        while time.monotonic() < deadline:
            self.step()
            if not paused and abs(self.command.linear.x) > 0.1:
                for _ in range(45):
                    self.step(scan=False)
                assert abs(self.command.linear.x) < 1e-6 and abs(self.command.angular.z) < 1e-6, 'scan watchdog failed'
                paused = True
            if self.trajectories and math.hypot(self.x - 3.0, self.y) < 0.2:
                break
        for _ in range(75):
            self.step()
        assert self.maps, 'no occupancy map'
        grid = self.maps[-1]
        assert grid.header.frame_id == 'odom' and grid.info.width == 400 and grid.info.height == 300
        assert grid.info.origin.orientation.w == 1.0 and any(v == 100 for v in grid.data)
        assert self.trajectories, 'no B-spline'
        assert paused, 'no motion to exercise watchdog'
        assert self.closest > 0.4, f'obstacle collision: {self.closest}'
        assert math.hypot(self.x - 3.0, self.y) < 0.2, f'goal not reached: {(self.x, self.y)}'
        assert abs(self.command.linear.x) < 1e-6 and abs(self.command.angular.z) < 1e-6, 'goal completion did not stop the base'
        for _ in range(60):
            self.step(obstacle=False)
        assert not any(value == 100 for value in self.maps[-1].data), 'free rays did not clear old obstacles'
        print(f'PASS: {len(self.trajectories)} trajectories, {len(self.commands)} bounded commands, clearance={self.closest:.3f}, goal={(self.x, self.y)}, scan watchdog stopped motion')


if __name__ == '__main__':
    rclpy.init()
    robot = Robot()
    try:
        robot.run()
    finally:
        robot.destroy_node()
        rclpy.shutdown()
