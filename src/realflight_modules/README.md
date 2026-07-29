# realflight_modules
https://wiki.t-firefly.com/zh_CN/ROC-RK3588S-PC/index.html

## 一键启动

```bash
./start_realflight.sh
```

脚本将依次启动 `rslidar_sdk`、MAVROS 和 FAST-LIO；按 `Ctrl+C` 可统一退出。

## PX4 EKF2 关键参数

 `px4bridge` 只发送**位置和姿态**。PX4
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

## 记录 odom 延迟

在 QGroundControl 中设置：

| 参数 | 设置 |
| --- | --- |
| `SDLOG_PROFILE` | 勾选 bit 7：`Computer Vision and Avoidance` |
| `SDLOG_MODE` | 台架测试时设为 2：`From boot until shutdown` |

`SDLOG_PROFILE` 是位掩码，不要覆盖原有选项。例如当前值为默认的 `1` 时，
增加 bit 7 后应为 `129`。模式 `2` 会持续记录到飞控关机；标定完成后可将
`SDLOG_MODE` 恢复为 `0`，减少日志量。

## 延迟测量与补偿

1. 拆桨或保持飞行器不解锁，启动 LIO、MAVROS 和 `px4bridge`。
2. 尽量保持机体水平，快速往复转动偏航角，然后下载 `.ulg` 日志。
3. 当前接口不发送角速度，因此 `vehicle_visual_odometry.angular_velocity`
   为 `0` 或无效是正常的，不要直接使用该字段。
4. 将 `vehicle_visual_odometry.q` 转为偏航角，展开正负 π 跳变后按相邻帧
   时间戳做差分，得到视觉偏航角速度；再与
   `vehicle_angular_velocity.xyz[2]` 对比，用波峰偏移或互相关计算时间差。
5. 若视觉曲线比 IMU 晚 `80 ms`，设置 `EKF2_EV_DELAY=80`，重启后复测。
6. 以 `5~10 ms` 为步长微调，使动态运动时的视觉 EKF innovation 最小。

`EKF2_EV_DELAY` 只能补偿固定延迟。LIO 必须使用采集时刻作为 odom 的
`header.stamp`，并保证 MAVROS 时间同步正常；如果延迟随时间明显变化，应先排查
LIO 计算、网络或串口抖动，而不是继续增大该参数。

Windows 下可使用脚本绘制两条角速度曲线：

```powershell
py -m pip install pyulog numpy matplotlib
py plot_ev_delay.py flight.ulg --start 10 --end 30
```
