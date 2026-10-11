# ROS 2 Humble：二维差速机器人

保留原有全局多项式、A*、B 样条优化、轨迹重规划与 MPC/qpOASES；通信、参数、消息生成和构建改用 ROS 2。新增 LaserScan 二维地图适配和差速底盘速度接口。下面是此分支的入口；原 README 的 Melodic/Gazebo、Patchwork 和旧 XML launch 属于 ROS 1，未迁移。

## 编译与启动

在 Ubuntu 22.04、ROS 2 Humble 环境中安装依赖：

```bash
sudo apt-get update
sudo apt-get install ros-humble-perception ros-humble-rviz2 ros-humble-ament-cmake-gtest python3-colcon-common-extensions python3-pytest
source /opt/ros/humble/setup.bash
mkdir -p ~/ego_ros2_ws
cd ~/ego_ros2_ws
colcon build --base-paths /path/to/ego-planner-for-ground-robot/ego-planner/planner --packages-up-to ego_planner --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch ego_planner ground_robot.launch.py
```

只编译 `ego-planner/planner`，避免把未迁移的 ROS 1 仿真和 Patchwork 加入 colcon。qpOASES 随仓库编译，不需要全局安装。无显示器时使用 `rviz:=false`。

## 机器人接口

| 输入/输出 | 类型 | 默认话题与约定 |
|---|---|---|
| 激光输入 | `sensor_msgs/msg/LaserScan` | `/scan`；传感器 QoS，正确时间戳和雷达 frame |
| 里程计输入 | `nav_msgs/msg/Odometry` | `/odom`；pose 在 header frame，twist 在车体 child frame |
| RViz 目标 | `geometry_msgs/msg/PoseStamped` | `/goal_pose`；使用 RViz 的 2D Goal Pose |
| 底盘输出 | `geometry_msgs/msg/Twist` | `/cmd_vel`；仅 linear.x、angular.z |
| 二维地图 | `nav_msgs/msg/OccupancyGrid` | `/map_2d`，包含障碍物膨胀 |
| 规划轨迹 | `ego_planner/msg/Bspline` | `/planning/bspline` |

机器人必须发布 `odom -> base_link -> 雷达frame` 的 TF。默认规划坐标系 `odom`；使用 `map` 时需要定位系统的 `map -> odom` TF。目标和里程计会转换到规划坐标系，并投影到二维。里程计 child frame 应与底盘运动坐标系一致。RViz 的目标在地图范围内且机器人已有里程计时才会接受。

```bash
ros2 launch ego_planner ground_robot.launch.py planning_frame:=map base_frame:=base_link scan_topic:=/scan odom_topic:=/odom cmd_vel_topic:=/cmd_vel
```

参数见 `ego-planner/planner/plan_manage/config/ground_robot.yaml`。默认地图 40×30 米、分辨率 0.1 米；线速度上限 1.0 m/s，角速度上限 1.2 rad/s。应按实际底盘能力、机器人尺寸和雷达安装修改膨胀半径、MPC 限制及 `stop_distance`。`sensor_timeout` 默认 0.5 秒：缺少有效雷达/里程计、数据超时或求解失败时输出零速度。仿真可用 `use_sim_time:=true`，并提供 `/clock`。

ROS 2 Humble 的 diff_drive_controller 若接受未加时间戳的 Twist，应设置 `use_stamped_vel: false`，并将此处输出重映射到该控制器的 `cmd_vel_unstamped`。

## 验证

```bash
cd ~/ego_ros2_ws
colcon test --base-paths /path/to/ego-planner-for-ground-robot/ego-planner/planner --packages-select ego_ros2 mpc_controller traj_utils
colcon test-result --verbose
```

闭环测试必须在没有真实底盘的独立 ROS domain 中运行。两个终端都设置相同 domain，先启动规划器，再运行合成机器人：

```bash
export ROS_DOMAIN_ID=42
ros2 launch ego_planner ground_robot.launch.py rviz:=false
# 第二终端也 source Humble 和工作区，并设置 ROS_DOMAIN_ID=42
python3 /path/to/ego-planner-for-ground-robot/ego-planner/planner/plan_manage/test/navigation_smoke.py
```

该测试发布带障碍物的雷达、TF 和差速里程计，通过带坐标变换的目标检查避障到达、速度限制、地图元数据及雷达超时停车。真实机器人性能仍需实机调参验证。
