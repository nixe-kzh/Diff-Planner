# realflight_modules

本目录包含实飞相关 ROS 功能包：

- `px4bridge`：按指定频率将 LIO 里程计位姿发送给 PX4。
- `px4ctrl`：PX4 控制相关功能。

## PX4 EKF2 关键参数

当前 `px4bridge` 只发送**位置和姿态**，不发送速度，也不携带有效协方差。PX4
1.14 及更新版本建议从以下配置开始：

| 参数 | 建议值 | 说明 |
| --- | ---: | --- |
| `EKF2_EV_CTRL` | `11` | 融合视觉水平位置、垂直位置和偏航角；不要开启速度融合 |
| `EKF2_EV_CTRL` | `3` | 如果 LIO 偏航角不可靠，改用此值，仅融合位置 |
| `EKF2_EV_NOISE_MD` | `1` | 使用 PX4 参数中的视觉噪声，不使用消息协方差 |
| `EKF2_EVP_NOISE` | `0.10` | 视觉位置噪声起始值，单位 m |
| `EKF2_EVA_NOISE` | `0.05` | 视觉姿态噪声起始值，单位 rad |
| `EKF2_EV_DELAY` | 实测值 | 视觉数据相对 IMU 的固定延迟，单位 ms |
| `EKF2_EV_POS_X/Y/Z` | `0` | 输入已是机体位姿时设为 0 |

其他按场景选择：

- 需要 LIO 的 Z 作为高度基准时，将 `EKF2_HGT_REF` 设为 `Vision`。
- 室内完全不使用 GNSS 时，可将 `EKF2_GPS_CTRL` 设为 `0`。
- 修改 EKF 参数后重启飞控。

> 旧版 PX4 没有 `EKF2_EV_CTRL` 时，在 `EKF2_AID_MASK` 中勾选
> `Vision position` 和需要的 `Vision yaw`，不要勾选 `Vision velocity`；
> 高度参考的旧参数名为 `EKF2_HGT_MODE`。

## 记录 odom 延迟

在 QGroundControl 中设置：

| 参数 | 设置 |
| --- | --- |
| `SDLOG_PROFILE` | 勾选 bit 7：`Computer Vision and Avoidance` |
| `SDLOG_MODE` | 台架测试时设为 `1`，从开机开始记录 |

`SDLOG_PROFILE` 是位掩码，不要覆盖原有选项。例如当前值为默认的 `1` 时，
增加 bit 7 后应为 `129`。标定完成后可将 `SDLOG_MODE` 恢复为 `0`，减少日志量。

## 延迟测量与补偿

1. 拆桨或保持飞行器不解锁，启动 LIO、MAVROS 和 `px4bridge`。
2. 快速往复转动偏航角并做短距离平移，然后下载 `.ulg` 日志。
3. 对比 `vehicle_visual_odometry.q` 求出的视觉偏航角速度与
   `vehicle_angular_velocity.xyz[2]`，用波峰偏移或互相关计算时间差。
4. 若视觉曲线比 IMU 晚 `80 ms`，设置 `EKF2_EV_DELAY=80`，重启后复测。
5. 以 `5~10 ms` 为步长微调，使动态运动时的视觉 EKF innovation 最小。

`EKF2_EV_DELAY` 只能补偿固定延迟。LIO 必须使用采集时刻作为 odom 的
`header.stamp`，并保证 MAVROS 时间同步正常；如果延迟随时间明显变化，应先排查
LIO 计算、网络或串口抖动，而不是继续增大该参数。

参考：[PX4 外部视觉位置估计](https://docs.px4.io/main/en/ros/external_position_estimation)、
[PX4 ULog 记录配置](https://docs.px4.io/main/en/dev_log/logging)
