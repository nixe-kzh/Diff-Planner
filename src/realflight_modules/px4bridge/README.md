# px4bridge

`px4bridge` 将高频 LIO 的 `nav_msgs/Odometry` 位姿限频后发布到
MAVROS 的 PX4 视觉位姿入口。默认输出话题为
`/mavros/vision_pose/pose`，消息类型为 `geometry_msgs/PoseStamped`。

MAVROS 会完成 ROS ENU/FLU 到 PX4 NED/FRD 的坐标变换，并将视觉位姿发送
给飞控。输入 odometry 应遵循 ROS 坐标约定，且其中的 pose 应表示机体位姿；
如果 LIO 输出的是雷达位姿，需要先应用雷达到机体的外参。

## 发布内容与 PX4 接收字段

本节点发布的是 `geometry_msgs/PoseStamped`，字段流向如下：

| LIO `nav_msgs/Odometry` | 本节点输出 | MAVLink `VISION_POSITION_ESTIMATE` | PX4 接收结果 |
| --- | --- | --- | --- |
| `header.stamp` | `header.stamp` | `usec`，单位微秒 | `vehicle_visual_odometry.timestamp_sample` |
| `pose.pose.position` | `pose.position` | `x/y/z`，MAVROS 将 ENU 转为 NED | 三轴局部位置 |
| `pose.pose.orientation` | 归一化后的 `pose.orientation` | `roll/pitch/yaw`，MAVROS 将 FLU/ENU 转为 FRD/NED | 姿态四元数 |
| `pose.covariance` | 不发布 | MAVROS 1.20.1 填零 | 未传递 LIO 协方差 |
| `twist` 与 `twist.covariance` | 不发布 | 此 MAVLink 消息没有这些字段 | 速度和角速度无效 |
| `child_frame_id` | 不发布 | 无对应字段 | 不参与转换 |

还需要注意：

- MAVROS 的 `vision_pose` 话题回调不会使用 `header.frame_id` 查询 TF；它直接
  假设输入是 ROS 约定的 ENU 世界系和 FLU 机体系。
- PX4 收到的位置已经是 NED，姿态已经转换为 PX4 使用的 FRD/NED 表达。
- `PoseStamped` 无法表达 `reset_counter`，MAVROS 默认发送 `0`。LIO
  回环导致位姿跳变时，PX4 无法通过当前接口得知这是一次估计器重置。
- 如果需要把 LIO 的位姿协方差传给 PX4，应改为发布
  `geometry_msgs/PoseWithCovarianceStamped` 到
  `/mavros/vision_pose/pose_cov`。
- 如果需要同时发送速度、角速度和两类协方差，应采用 MAVROS odometry
  接口，而不是 `vision_pose/pose`。

## 编译

```bash
cd ~/nanobot_ws
catkin_make --pkg px4bridge
source devel/setup.bash
```

## 启动

```bash
roslaunch px4bridge px4bridge.launch \
  odom_topic:=/lio/odom \
  vision_pose_rate_hz:=50.0
```

可用的 launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `odom_topic` | `/Odometry` | LIO odometry 输入话题 |
| `vision_pose_topic` | `/mavros/vision_pose/pose` | MAVROS 视觉位姿输入话题 |
| `vision_pose_rate_hz` | `50.0` | 有新 odometry 时的最大发送频率，单位 Hz |
| `odometry_timeout_sec` | `0.2` | 超过此时间未收到 odometry 时暂停发送 |

## PX4 配置提示

确保 MAVROS 已连接 PX4，并在 PX4 EKF2 参数中启用外部视觉位置/姿态融合。
具体参数取决于 PX4 固件版本。实飞前可检查：

```bash
rostopic hz /mavros/vision_pose/pose
rostopic echo -n 1 /mavros/vision_pose/pose
```
