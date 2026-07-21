#!/usr/bin/env python3
"""Collects thrust-command and battery-voltage samples for calibration."""

import csv
import time
from pathlib import Path
from typing import List, Tuple

import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import BatteryState


_BATTERY_TOPIC = "/mavros/battery"
_THRUST_COMMAND_TOPIC = "/mavros/setpoint_raw/attitude"
_TRIGGER_TOPIC = "/traj_start_trigger"
_OUTPUT_FILENAME = "data.csv"


class ThrustCalibrationRecorder:
    """Records averaged thrust commands and battery voltages to a CSV file."""

    def __init__(self) -> None:
        """Loads parameters and creates the ROS subscribers."""
        self._average_interval = float(rospy.get_param("~time_interval", 1.0))
        self._minimum_voltage = float(
            rospy.get_param("~min_battery_voltage", 13.2)
        )
        self._mass_kg = float(rospy.get_param("~mass_kg", 1.0))

        if self._average_interval <= 0.0:
            raise ValueError("~time_interval must be greater than zero")

        self._voltage_buffer: List[float] = []
        self._thrust_buffer: List[float] = []
        self._records: List[Tuple[float, float]] = []
        self._last_record_time = rospy.get_rostime()
        self._recording = False
        self._saved = False

        self._battery_subscriber = rospy.Subscriber(
            _BATTERY_TOPIC, BatteryState, self._battery_voltage_callback
        )
        self._thrust_subscriber = rospy.Subscriber(
            _THRUST_COMMAND_TOPIC,
            AttitudeTarget,
            self._thrust_command_callback,
        )
        self._trigger_subscriber = rospy.Subscriber(
            _TRIGGER_TOPIC, PoseStamped, self._trigger_callback
        )

    def _battery_voltage_callback(self, message: BatteryState) -> None:
        """Collects voltage samples and periodically records an average."""
        if not self._recording:
            return

        self._voltage_buffer.append(float(message.voltage))
        current_time = rospy.get_rostime()
        elapsed_time = (current_time - self._last_record_time).to_sec()
        if elapsed_time >= self._average_interval:
            self._record_average(current_time)

        if message.voltage < self._minimum_voltage:
            self._finish_recording(
                "battery voltage {:.3f} V is below {:.3f} V".format(
                    message.voltage, self._minimum_voltage
                )
            )

    def _thrust_command_callback(self, message: AttitudeTarget) -> None:
        """Collects normalized thrust-command samples while recording."""
        if self._recording:
            self._thrust_buffer.append(float(message.thrust))

    def _trigger_callback(self, _message: PoseStamped) -> None:
        """Starts recording on the first trigger and stops on the second."""
        if self._saved:
            return

        if not self._recording:
            self._recording = True
            self._last_record_time = rospy.get_rostime()
            rospy.loginfo("Thrust calibration recording started")
            return

        self._finish_recording("stop trigger received")

    def _record_average(self, record_time: rospy.Time) -> None:
        """Stores one averaged sample when both input buffers have data."""
        if not self._voltage_buffer or not self._thrust_buffer:
            rospy.logwarn_throttle(
                1.0, "Waiting for synchronized voltage and thrust samples"
            )
            return

        average_voltage = sum(self._voltage_buffer) / len(self._voltage_buffer)
        average_thrust = sum(self._thrust_buffer) / len(self._thrust_buffer)
        self._records.append((average_thrust, average_voltage))
        self._voltage_buffer.clear()
        self._thrust_buffer.clear()
        self._last_record_time = record_time

        rospy.loginfo(
            "Calibration sample: thrust=%.4f, voltage=%.3f V",
            average_thrust,
            average_voltage,
        )

    def _finish_recording(self, reason: str) -> None:
        """Stops data collection and writes all samples to disk."""
        if self._saved:
            return

        self._recording = False
        if self._voltage_buffer and self._thrust_buffer:
            self._record_average(rospy.get_rostime())

        output_path = self._save_data()
        self._saved = True
        self._battery_subscriber.unregister()
        self._thrust_subscriber.unregister()
        self._trigger_subscriber.unregister()

        rospy.loginfo("Thrust calibration stopped: %s", reason)
        rospy.loginfo("Saved %d samples to %s", len(self._records), output_path)
        rospy.signal_shutdown("thrust calibration complete")

    def _save_data(self) -> Path:
        """Appends calibration metadata and paired samples to the CSV file."""
        package_path = Path(rospkg.RosPack().get_path("px4ctrl"))
        output_path = package_path / "scripts" / _OUTPUT_FILENAME

        with output_path.open("a", encoding="utf-8", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                (
                    time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
                    "mass_kg",
                    self._mass_kg,
                    "thrust_command",
                    "voltage_v",
                )
            )
            writer.writerows(self._records)

        return output_path


def main() -> None:
    """Runs the thrust calibration recorder node."""
    rospy.init_node("thrust_calibration")
    ThrustCalibrationRecorder()
    rospy.loginfo("Waiting for thrust calibration trigger")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, rospy.ROSInitException):
        pass
