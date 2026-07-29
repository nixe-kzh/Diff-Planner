# firefly web
https://wiki.t-firefly.com/zh_CN/ROC-RK3588S-PC/index.html


## PX4 EKF2 关键参数

 `px4bridge` 只发送**位置和姿态**

| 参数 | 建议值 | 说明 |
| --- | ---: | --- |
| `EKF2_EV_CTRL` | `11` | 融合视觉水平位置、垂直位置和偏航角；不要开启速度融合 |
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

标定完成后可将`SDLOG_MODE` 恢复为 `0`，减少日志量。

`EKF2_EV_DELAY` 只能补偿固定延迟, 使用 `src/realflight_modules/px4bridge/scripts/plot_ev_delay.py`来绘制ulg数据包里面的vision角速度和imu角速度来获取ekf delay。
