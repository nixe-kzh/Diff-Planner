#!/usr/bin/env python3
"""Plot position, velocity and attitude tracking from a ROS1 bag.

All settings are defined in this file. No command-line arguments are needed.
"""

import math
import sys
from pathlib import Path


# ============================ Configuration ==============================
BAG_PATH = Path("/home/nx/2026-08-04-03-07-22.bag")
OUTPUT_DIR = BAG_PATH.parent / ("tracking_analysis_" + BAG_PATH.stem)

# True: save figures and open plot windows; False: only save figures.
VISUAL = False #! 是否可视化图窗

SETPOINT_TOPIC = "/setpoints_cmd"
ODOMETRY_TOPIC = "/Odometry_high_rate"
ATTITUDE_SETPOINT_TOPIC = "/mavros/setpoint_raw/attitude"
ATTITUDE_FEEDBACK_TOPIC = "/mavros/imu/data"



USE_HEADER_TIMESTAMP = True
PLOT_ONLY_SETPOINT_INTERVAL = True
PLOT_PADDING_S = 0.50

FIGURE_DPI = 170
ANGLE_UNIT = "deg"  # "deg" or "rad"

# Allow the script to run directly from a plain terminal on this machine.
ROS_PYTHON_PATHS = (
    "/opt/ros/noetic/lib/python3/dist-packages",
)
# ========================================================================


for ros_python_path in reversed(ROS_PYTHON_PATHS):
    if ros_python_path not in sys.path and Path(ros_python_path).is_dir():
        sys.path.insert(0, ros_python_path)

try:
    import rosbag
except ImportError as exc:
    raise SystemExit(
        "Cannot import rosbag. Check ROS_PYTHON_PATHS or source the ROS Noetic "
        "environment first.\n{}".format(exc)
    )

import numpy as np

if not VISUAL:
    import matplotlib

    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401; registers "3d" projection


def message_time(msg, bag_time):
    """Use message header time when available, otherwise use bag record time."""
    if USE_HEADER_TIMESTAMP and hasattr(msg, "header"):
        stamp = msg.header.stamp
        if stamp.secs != 0 or stamp.nsecs != 0:
            return float(stamp.to_sec())
    return float(bag_time.to_sec())


def quaternion_to_euler(x, y, z, w):
    """Convert a quaternion to roll, pitch and yaw in radians."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        return math.nan, math.nan, math.nan
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_argument = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_argument)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def empty_data():
    return {
        "setpoint": {
            "time": [],
            "position": [],
            "velocity": [],
        },
        "odometry": {
            "time": [],
            "position": [],
            "velocity": [],
        },
        "attitude_setpoint": {
            "time": [],
            "euler": [],
        },
        "attitude_feedback": {
            "time": [],
            "euler": [],
        },
    }


def read_bag(path):
    if not path.is_file():
        raise FileNotFoundError("Bag file not found: {}".format(path))

    data = empty_data()
    topics = (
        SETPOINT_TOPIC,
        ODOMETRY_TOPIC,
        ATTITUDE_SETPOINT_TOPIC,
        ATTITUDE_FEEDBACK_TOPIC,
    )

    print("Reading {} ...".format(path))
    with rosbag.Bag(str(path), "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=topics):
            timestamp = message_time(msg, bag_time)

            if topic == SETPOINT_TOPIC:
                item = data["setpoint"]
                item["time"].append(timestamp)
                item["position"].append(
                    [msg.position.x, msg.position.y, msg.position.z]
                )
                item["velocity"].append(
                    [msg.velocity.x, msg.velocity.y, msg.velocity.z]
                )

            elif topic == ODOMETRY_TOPIC:
                item = data["odometry"]
                position = msg.pose.pose.position
                velocity = msg.twist.twist.linear
                item["time"].append(timestamp)
                item["position"].append(
                    [position.x, position.y, position.z]
                )
                item["velocity"].append(
                    [velocity.x, velocity.y, velocity.z]
                )

            elif topic == ATTITUDE_SETPOINT_TOPIC:
                quaternion = msg.orientation
                item = data["attitude_setpoint"]
                item["time"].append(timestamp)
                item["euler"].append(
                    quaternion_to_euler(
                        quaternion.x,
                        quaternion.y,
                        quaternion.z,
                        quaternion.w,
                    )
                )

            elif topic == ATTITUDE_FEEDBACK_TOPIC:
                quaternion = msg.orientation
                item = data["attitude_feedback"]
                item["time"].append(timestamp)
                item["euler"].append(
                    quaternion_to_euler(
                        quaternion.x,
                        quaternion.y,
                        quaternion.z,
                        quaternion.w,
                    )
                )

    return finalize_data(data)


def finalize_data(data):
    """Convert lists to sorted arrays and set the first setpoint as t=0."""
    for item in data.values():
        for field, values in item.items():
            item[field] = np.asarray(values, dtype=float)

        if len(item["time"]) == 0:
            continue
        order = np.argsort(item["time"])
        for field in item:
            item[field] = item[field][order]

    if len(data["setpoint"]["time"]) == 0:
        raise RuntimeError("No messages found on {}".format(SETPOINT_TOPIC))

    time_origin = float(data["setpoint"]["time"][0])
    for item in data.values():
        item["time"] = item["time"] - time_origin
    return data


def validate_topics(data):
    configured_topics = (
        ("setpoint", SETPOINT_TOPIC),
        ("odometry", ODOMETRY_TOPIC),
        ("attitude_setpoint", ATTITUDE_SETPOINT_TOPIC),
        ("attitude_feedback", ATTITUDE_FEEDBACK_TOPIC),
    )
    for name, topic in configured_topics:
        if len(data[name]["time"]) == 0:
            raise RuntimeError("Required topic has no messages: {}".format(topic))


def plot_limits(data):
    if PLOT_ONLY_SETPOINT_INTERVAL:
        times = data["setpoint"]["time"]
        return float(times[0] - PLOT_PADDING_S), float(times[-1] + PLOT_PADDING_S)

    starts = [float(item["time"][0]) for item in data.values()]
    ends = [float(item["time"][-1]) for item in data.values()]
    return min(starts), max(ends)


def in_window(times, limits):
    return (times >= limits[0]) & (times <= limits[1])


def unwrap_euler(euler):
    result = np.asarray(euler, dtype=float).copy()
    for column in range(3):
        result[:, column] = np.unwrap(result[:, column])
    return result


def angle_scale():
    return 180.0 / math.pi if ANGLE_UNIT == "deg" else 1.0


def wrap_angle(values):
    return np.arctan2(np.sin(values), np.cos(values))


def interpolate(feedback_time, feedback_values, reference_time):
    feedback_values = np.asarray(feedback_values, dtype=float)
    return np.column_stack(
        [
            np.interp(
                reference_time,
                feedback_time,
                feedback_values[:, column],
            )
            for column in range(feedback_values.shape[1])
        ]
    )


def matched_error(
    reference_time,
    reference_values,
    feedback_time,
    feedback_values,
    angular=False,
):
    """Calculate reference minus time-interpolated feedback."""
    valid = (
        (reference_time >= feedback_time[0])
        & (reference_time <= feedback_time[-1])
    )
    matched_time = reference_time[valid]
    reference = reference_values[valid]
    feedback = interpolate(feedback_time, feedback_values, matched_time)
    error = reference - feedback
    return wrap_angle(error) if angular else error


def metrics(values):
    result = metric_values(values)
    return result["rmse"], result["mae"], result["max_abs_error"]


def metric_values(values):
    """Return scalar error statistics for plots and the TOML report."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return {
            "sample_count": 0,
            "mean_error": math.nan,
            "std_error": math.nan,
            "mse": math.nan,
            "rmse": math.nan,
            "mae": math.nan,
            "max_abs_error": math.nan,
        }

    mse = float(np.mean(values * values))
    return {
        "sample_count": int(len(values)),
        "mean_error": float(np.mean(values)),
        "std_error": float(np.std(values)),
        "mse": mse,
        "rmse": float(math.sqrt(mse)),
        "mae": float(np.mean(np.abs(values))),
        "max_abs_error": float(np.max(np.abs(values))),
    }


def style_axis(axis, ylabel, limits):
    axis.set_ylabel(ylabel)
    axis.set_xlim(limits)
    axis.grid(True, alpha=0.28, linewidth=0.8)


def save_figure(figure, filename):
    output_path = OUTPUT_DIR / filename
    figure.savefig(str(output_path), dpi=FIGURE_DPI, bbox_inches="tight")
    print("Saved {}".format(output_path))


def plot_xyz_tracking(data, limits, field, unit, filename, title):
    reference = data["setpoint"]
    feedback = data["odometry"]
    reference_mask = in_window(reference["time"], limits)
    feedback_mask = in_window(feedback["time"], limits)

    figure, axes = plt.subplots(
        3,
        1,
        figsize=(12, 8.3),
        sharex=True,
        constrained_layout=True,
    )
    figure.suptitle(title, fontsize=14)

    for column, channel in enumerate(("x", "y", "z")):
        axis = axes[column]
        axis.plot(
            feedback["time"][feedback_mask],
            feedback[field][feedback_mask, column],
            color="tab:blue",
            linewidth=1.25,
            label="measured",
        )
        axis.plot(
            reference["time"][reference_mask],
            reference[field][reference_mask, column],
            color="tab:orange",
            linestyle="--",
            linewidth=1.55,
            label="setpoint",
        )

        error = matched_error(
            reference["time"][reference_mask],
            reference[field][reference_mask],
            feedback["time"][feedback_mask],
            feedback[field][feedback_mask],
        )[:, column]
        rmse, mae, maximum = metrics(error)
        axis.set_title(
            "{} channel | RMSE={:.4f}, MAE={:.4f}, max|e|={:.4f}".format(
                channel,
                rmse,
                mae,
                maximum,
            ),
            loc="left",
            fontsize=10,
        )
        style_axis(axis, "{} [{}]".format(channel, unit), limits)
        axis.legend(loc="best", fontsize=9)

    axes[-1].set_xlabel("time from first setpoint [s]")
    save_figure(figure, filename)


def plot_attitude_tracking(data, limits):
    desired = data["attitude_setpoint"]
    measured = data["attitude_feedback"]
    desired_mask = in_window(desired["time"], limits)
    measured_mask = in_window(measured["time"], limits)
    desired_euler = unwrap_euler(desired["euler"])
    measured_euler = unwrap_euler(measured["euler"])
    scale = angle_scale()

    figure, axes = plt.subplots(
        3,
        1,
        figsize=(12, 8.3),
        sharex=True,
        constrained_layout=True,
    )
    figure.suptitle(
        "Attitude tracking (MAVROS ENU/FLU convention)",
        fontsize=14,
    )

    for column, channel in enumerate(("roll", "pitch", "yaw")):
        axis = axes[column]
        axis.plot(
            measured["time"][measured_mask],
            measured_euler[measured_mask, column] * scale,
            color="tab:blue",
            linewidth=1.15,
            label="IMU measured",
        )
        axis.plot(
            desired["time"][desired_mask],
            desired_euler[desired_mask, column] * scale,
            color="tab:orange",
            linestyle="--",
            linewidth=1.45,
            label="attitude setpoint",
        )

        error = matched_error(
            desired["time"][desired_mask],
            desired_euler[desired_mask],
            measured["time"][measured_mask],
            measured_euler[measured_mask],
            angular=True,
        )[:, column]
        rmse, mae, maximum = metrics(error * scale)
        axis.set_title(
            "{} | RMSE={:.3f}, MAE={:.3f}, max|e|={:.3f} {}".format(
                channel,
                rmse,
                mae,
                maximum,
                ANGLE_UNIT,
            ),
            loc="left",
            fontsize=10,
        )
        style_axis(
            axis,
            "{} [{}]".format(channel, ANGLE_UNIT),
            limits,
        )
        axis.legend(loc="best", fontsize=9)

    axes[-1].set_xlabel("time from first setpoint [s]")
    save_figure(figure, "03_attitude_tracking.png")


def set_axes_equal_3d(axis, points):
    """Use equal scaling on all axes so the 3D trajectory is not distorted."""
    finite_points = points[np.all(np.isfinite(points), axis=1)]
    if len(finite_points) == 0:
        return

    minimum = np.min(finite_points, axis=0)
    maximum = np.max(finite_points, axis=0)
    center = 0.5 * (minimum + maximum)
    radius = 0.5 * float(np.max(maximum - minimum))
    if radius < 1.0e-9:
        radius = 0.5

    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)


def plot_3d_position_tracking(data, limits):
    """Plot the desired and measured XYZ trajectories in one 3D figure."""
    reference = data["setpoint"]
    feedback = data["odometry"]
    reference_mask = in_window(reference["time"], limits)
    feedback_mask = in_window(feedback["time"], limits)
    reference_position = reference["position"][reference_mask]
    feedback_position = feedback["position"][feedback_mask]

    figure = plt.figure(figsize=(10, 8), constrained_layout=True)
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(
        feedback_position[:, 0],
        feedback_position[:, 1],
        feedback_position[:, 2],
        color="tab:blue",
        linewidth=1.35,
        label="measured",
    )
    axis.plot(
        reference_position[:, 0],
        reference_position[:, 1],
        reference_position[:, 2],
        color="tab:orange",
        linestyle="--",
        linewidth=1.65,
        label="setpoint",
    )

    axis.set_title("3D position tracking", fontsize=14)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_zlabel("z [m]")
    axis.grid(True, alpha=0.28, linewidth=0.8)
    axis.legend(loc="best")
    set_axes_equal_3d(
        axis,
        np.vstack((reference_position, feedback_position)),
    )
    save_figure(figure, "04_position_tracking_3d.png")


def toml_string(value):
    return '"{}"'.format(str(value).replace("\\", "\\\\").replace('"', '\\"'))


def append_metric_section(lines, section, unit, values):
    result = metric_values(values)
    lines.extend(
        [
            "",
            "[{}]".format(section),
            "unit = {}".format(toml_string(unit)),
            "sample_count = {}".format(result["sample_count"]),
            "mean_error = {:.12g}".format(result["mean_error"]),
            "std_error = {:.12g}".format(result["std_error"]),
            "mse = {:.12g}".format(result["mse"]),
            "rmse = {:.12g}".format(result["rmse"]),
            "mae = {:.12g}".format(result["mae"]),
            "max_abs_error = {:.12g}".format(result["max_abs_error"]),
        ]
    )


def write_metrics_toml(data, limits):
    """Write synchronized per-channel tracking statistics as valid TOML."""
    setpoint = data["setpoint"]
    odometry = data["odometry"]
    attitude_setpoint = data["attitude_setpoint"]
    attitude_feedback = data["attitude_feedback"]

    setpoint_mask = in_window(setpoint["time"], limits)
    odometry_mask = in_window(odometry["time"], limits)
    attitude_setpoint_mask = in_window(attitude_setpoint["time"], limits)
    attitude_feedback_mask = in_window(attitude_feedback["time"], limits)

    position_error = matched_error(
        setpoint["time"][setpoint_mask],
        setpoint["position"][setpoint_mask],
        odometry["time"][odometry_mask],
        odometry["position"][odometry_mask],
    )
    velocity_error = matched_error(
        setpoint["time"][setpoint_mask],
        setpoint["velocity"][setpoint_mask],
        odometry["time"][odometry_mask],
        odometry["velocity"][odometry_mask],
    )

    desired_euler = unwrap_euler(attitude_setpoint["euler"])
    measured_euler = unwrap_euler(attitude_feedback["euler"])
    attitude_error = matched_error(
        attitude_setpoint["time"][attitude_setpoint_mask],
        desired_euler[attitude_setpoint_mask],
        attitude_feedback["time"][attitude_feedback_mask],
        measured_euler[attitude_feedback_mask],
        angular=True,
    ) * angle_scale()

    lines = [
        "# Empirical tracking-error statistics: error = setpoint - measured.",
        "# MSE is the empirical mean squared error over synchronized samples.",
        "[meta]",
        "bag_path = {}".format(toml_string(BAG_PATH)),
        "plot_start_s = {:.12g}".format(limits[0]),
        "plot_end_s = {:.12g}".format(limits[1]),
        "timestamp_source = {}".format(
            toml_string("message_header" if USE_HEADER_TIMESTAMP else "bag_record")
        ),
        "angle_unit = {}".format(toml_string(ANGLE_UNIT)),
        "",
        "[topics]",
        "setpoint = {}".format(toml_string(SETPOINT_TOPIC)),
        "odometry = {}".format(toml_string(ODOMETRY_TOPIC)),
        "attitude_setpoint = {}".format(toml_string(ATTITUDE_SETPOINT_TOPIC)),
        "attitude_feedback = {}".format(toml_string(ATTITUDE_FEEDBACK_TOPIC)),
        "",
        "[message_counts]",
        "setpoint = {}".format(len(setpoint["time"])),
        "odometry = {}".format(len(odometry["time"])),
        "attitude_setpoint = {}".format(len(attitude_setpoint["time"])),
        "attitude_feedback = {}".format(len(attitude_feedback["time"])),
    ]

    for field, errors, unit, channels in (
        ("position", position_error, "m", ("x", "y", "z")),
        ("velocity", velocity_error, "m/s", ("x", "y", "z")),
        ("attitude", attitude_error, ANGLE_UNIT, ("roll", "pitch", "yaw")),
    ):
        for column, channel in enumerate(channels):
            append_metric_section(
                lines,
                "{}.{}".format(field, channel),
                unit,
                errors[:, column],
            )
        append_metric_section(
            lines,
            "{}.error_norm".format(field),
            unit,
            np.linalg.norm(errors, axis=1),
        )

    output_path = OUTPUT_DIR / "tracking_metrics.toml"
    output_path.write_text("\n".join(lines) + "\n")
    print("Saved {}".format(output_path))


def main():
    if ANGLE_UNIT not in ("deg", "rad"):
        raise ValueError("ANGLE_UNIT must be 'deg' or 'rad'.")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    data = read_bag(BAG_PATH)
    validate_topics(data)
    limits = plot_limits(data)

    print(
        "Extracted messages: setpoint={}, odometry={}, attitude_setpoint={}, "
        "attitude_feedback={}".format(
            len(data["setpoint"]["time"]),
            len(data["odometry"]["time"]),
            len(data["attitude_setpoint"]["time"]),
            len(data["attitude_feedback"]["time"]),
        )
    )
    print("Plot interval: {:.3f} to {:.3f} s".format(*limits))

    plot_xyz_tracking(
        data,
        limits,
        "position",
        "m",
        "01_position_tracking.png",
        "Position tracking",
    )
    plot_xyz_tracking(
        data,
        limits,
        "velocity",
        "m/s",
        "02_velocity_tracking.png",
        "Velocity tracking",
    )
    plot_attitude_tracking(data, limits)
    plot_3d_position_tracking(data, limits)
    write_metrics_toml(data, limits)

    if VISUAL:
        plt.show()
    else:
        plt.close("all")
    print("Done. Results are in {}".format(OUTPUT_DIR))


if __name__ == "__main__":
    main()
