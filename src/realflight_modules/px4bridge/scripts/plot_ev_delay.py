#!/usr/bin/env python3

"""绘制 PX4 ULog 中的视觉偏航角速度和 IMU Z 轴角速度。"""

import argparse
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import numpy as np
    from pyulog import ULog
except ImportError as error:
    raise SystemExit(
        "缺少依赖运行 pip install pyulog numpy matplotlib"
    ) from error


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="对比外部视觉偏航角速度与 IMU Z 轴角速度。"
    )
    parser.add_argument("ulg_file", type=Path, help="PX4 .ulg 日志路径")
    parser.add_argument(
        "--start", type=float, default=None, help="绘图起始时间，单位 s"
    )
    parser.add_argument(
        "--end", type=float, default=None, help="绘图结束时间，单位 s"
    )
    parser.add_argument(
        "--smooth",
        type=int,
        default=5,
        help="视觉角速度的居中平滑窗口，默认 5，设为 1 表示不平滑",
    )
    parser.add_argument("--save", type=Path, help="保存图片，不指定则显示窗口")
    return parser.parse_args()


def get_sample_time(data):
    """优先使用采样时间；旧日志没有该字段时使用消息时间。"""
    field_name = "timestamp_sample" if "timestamp_sample" in data else "timestamp"
    return np.asarray(data[field_name], dtype=np.float64) * 1e-6


def keep_increasing_samples(sample_time, *values):
    """排序并删除重复或倒退时间戳，避免求导时除以零。"""
    order = np.argsort(sample_time)
    sorted_time = sample_time[order]
    sorted_values = [value[order] for value in values]

    keep = np.concatenate(([True], np.diff(sorted_time) > 0.0))
    return (sorted_time[keep], *(value[keep] for value in sorted_values))


def quaternion_to_yaw(w, x, y, z):
    """将 PX4 使用的 [w, x, y, z] 四元数转换为偏航角。"""
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    valid = norm > 1e-12
    if not np.all(valid):
        raise ValueError("vehicle_visual_odometry.q 中存在无效四元数")

    w = w / norm
    x = x / norm
    y = y / norm
    z = z / norm
    return np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def centered_moving_average(values, window_size):
    """居中平均不会像单向低通滤波那样额外引入时间延迟。"""
    if window_size <= 1:
        return values
    if window_size % 2 == 0:
        raise ValueError("--smooth 必须是奇数")
    if window_size > len(values):
        raise ValueError("--smooth 不能大于视觉数据点数量")

    kernel = np.ones(window_size, dtype=np.float64) / window_size
    return np.convolve(values, kernel, mode="same")


def main():
    args = parse_arguments()
    if not args.ulg_file.is_file():
        raise FileNotFoundError(f"找不到日志文件：{args.ulg_file}")

    ulog = ULog(str(args.ulg_file))
    visual_data = ulog.get_dataset("vehicle_visual_odometry").data
    imu_data = ulog.get_dataset("vehicle_angular_velocity").data

    visual_time = get_sample_time(visual_data)
    visual_time, q0, q1, q2, q3 = keep_increasing_samples(
        visual_time,
        np.asarray(visual_data["q[0]"]),
        np.asarray(visual_data["q[1]"]),
        np.asarray(visual_data["q[2]"]),
        np.asarray(visual_data["q[3]"]),
    )
    if len(visual_time) < 3:
        raise ValueError("有效视觉数据不足，请检查日志记录选项和时间戳")

    # PX4 四元数顺序为 [w, x, y, z]。先展开 ±π 跳变，再对时间求导。
    visual_yaw = np.unwrap(quaternion_to_yaw(q0, q1, q2, q3))
    visual_yaw_rate = np.gradient(visual_yaw, visual_time)
    visual_yaw_rate = centered_moving_average(visual_yaw_rate, args.smooth)

    imu_time = get_sample_time(imu_data)
    imu_time, imu_yaw_rate = keep_increasing_samples(
        imu_time, np.asarray(imu_data["xyz[2]"])
    )
    if len(imu_time) < 3:
        raise ValueError("有效 IMU 数据不足，请检查 ULog 内容")

    time_origin = min(visual_time[0], imu_time[0])
    visual_time -= time_origin
    imu_time -= time_origin

    visual_mask = np.ones(visual_time.shape, dtype=bool)
    imu_mask = np.ones(imu_time.shape, dtype=bool)
    if args.start is not None:
        visual_mask &= visual_time >= args.start
        imu_mask &= imu_time >= args.start
    if args.end is not None:
        visual_mask &= visual_time <= args.end
        imu_mask &= imu_time <= args.end

    plt.figure(figsize=(12, 6))
    plt.plot(
        imu_time[imu_mask],
        imu_yaw_rate[imu_mask],
        label="IMU: vehicle_angular_velocity.xyz[2]",
        linewidth=1.0,
    )
    plt.plot(
        visual_time[visual_mask],
        visual_yaw_rate[visual_mask],
        label="Vision: derivative of vehicle_visual_odometry.q",
        linewidth=1.5,
    )
    plt.xlabel("Time from log start (s)")
    plt.ylabel("Yaw rate (rad/s)")
    plt.title("External Vision Delay Check")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=160)
        print(f"图片已保存：{args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
