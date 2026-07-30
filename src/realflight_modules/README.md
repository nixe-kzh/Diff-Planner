# firefly web
https://wiki.t-firefly.com/zh_CN/ROC-RK3588S-PC/index.html

# realflight

依赖安装:
```
#PCL
sudo apt install -y libpcl-dev
#Ompl
sudo apt install -y libompl-dev ompl-demos
#Egien
sudo apt install -y libeigen3-dev
# Glog
sudo apt install -y libgoogle-glog-dev
# Fmt
sudo apt install -y libfmt-dev
sudo apt install -y ros-noetic-rosfmt

sudo apt install -y build-essential cmake
sudo apt install -y libpcap-dev
sudo apt install -y \
  git cmake build-essential pkg-config \
  libapr1-dev libaprutil1-dev \
  libboost-all-dev \
  ros-${ROS_DISTRO}-pcl-ros \
  ros-${ROS_DISTRO}-pcl-conversions
```

mavros:
```
sudo apt update
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras 

```

# LIO

```

git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build && cd build
cmake .. && make -j
sudo make install
```


```
# Diff planner
git clone -b dev_nanobot https://github.com/zhan994/Diff-Planner.git
# 雷达驱动
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git ~/Diff-Planner/src/realflight_modules/rslidar_sdk

cd ~/Diff-Planner/src/realflight_modules/rslidar_sdk
git submodule init
git submodule update

# FAST-LIO
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ~/Diff-Planner/src/realflight_modules/livox_ros_driver2
git clone -b dev_nanobot https://github.com/zhan994/FAST_LIO.git ~/Diff-Planner/src/realflight_modules/FAST_LIO
```

编译：
```
cd Diff-Planner
catkin_make
```
# 雷达驱动修改

- 指定雷达型号

修改 config 文件夹的 config.yaml 参数

`lidar_type: RSAIRY `


- 指定IMU 端口

修改 config 文件夹的 config.yaml 参数

`imu_port: 6688`

- 开启IMU

修改CMakeList参数：

```
ENABLE_IMU_DATA_PARSE = ON
```
- 设置点云格式：
修改CMakeList参数：
```
set(POINT_TYPE XYZIRT)
```
- IP设置：

先确认 NetworkManager 是否可用：
```
command -v nmcli
```
有输出的话，执行下面
```
sudo nmcli connection add \
  type ethernet \
  ifname eth0 \
  con-name lidar-static \
  ipv4.method manual \
  ipv4.addresses 192.168.1.102/24 \
  ipv4.never-default yes \
  ipv6.method disabled \
  connection.autoconnect yes
```
启用配置：
```
sudo nmcli connection up lidar-static
```
检查：
```
ip -br addr
ip route
```
正常应类似：
```
eth0   UP   192.168.1.102/24
wlan0  UP   192.168.8.238/24
```

# PX4 EKF2 关键参数

## px4bridge
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
