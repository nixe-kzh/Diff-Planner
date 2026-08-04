#!/usr/bin/env python3
"""Accumulate a ROS PointCloud2 stream into a height-filtered voxel cloud.

The input cloud must already be expressed in a fixed/global coordinate frame.
Points from every message are accumulated until this node exits.
"""

import math

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


# ============================ Configuration ==============================
INPUT_TOPIC = "/diff_planner_node/grid_map/occupancy_inflate"
OUTPUT_TOPIC = "/diff_planner_node/grid_map/occupancy_inflate_accumulated"
OUTPUT_RESOLUTION = 0.2  # Voxel side length [m].
HEIGHT_MIN = -1.0  # Keep points whose z is at least this value [m].
HEIGHT_MAX = 1.6  # Keep points whose z is at most this value [m].
# ========================================================================


class PointCloudAccumulator:
    """Height-filter, voxelize, accumulate and republish XYZ point clouds."""

    def __init__(self):
        if OUTPUT_RESOLUTION <= 0.0:
            raise ValueError("OUTPUT_RESOLUTION must be greater than zero")
        if HEIGHT_MIN > HEIGHT_MAX:
            raise ValueError("HEIGHT_MIN must not be greater than HEIGHT_MAX")

        # Integer voxel indices avoid floating-point equality problems in set.
        self.voxels = set()
        self.frame_id = None
        self.publisher = rospy.Publisher(
            OUTPUT_TOPIC,
            PointCloud2,
            queue_size=1,
            latch=True,
        )
        self.subscriber = rospy.Subscriber(
            INPUT_TOPIC,
            PointCloud2,
            self.cloud_callback,
            queue_size=1,
            buff_size=64 * 1024 * 1024,
        )

        rospy.loginfo(
            "Accumulating %s -> %s, resolution=%.3f m, z=[%.3f, %.3f] m",
            INPUT_TOPIC,
            OUTPUT_TOPIC,
            OUTPUT_RESOLUTION,
            HEIGHT_MIN,
            HEIGHT_MAX,
        )

    def cloud_callback(self, message):
        if self.frame_id is None:
            self.frame_id = message.header.frame_id
        elif message.header.frame_id != self.frame_id:
            rospy.logerr_throttle(
                5.0,
                "Ignoring point cloud in frame '%s'; accumulated cloud uses "
                "frame '%s'. Transform the input into one fixed frame first."
                % (message.header.frame_id, self.frame_id),
            )
            return

        inverse_resolution = 1.0 / OUTPUT_RESOLUTION
        previous_count = len(self.voxels)

        for x, y, z in point_cloud2.read_points(
            message,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < HEIGHT_MIN or z > HEIGHT_MAX:
                continue

            self.voxels.add(
                (
                    int(math.floor(x * inverse_resolution)),
                    int(math.floor(y * inverse_resolution)),
                    int(math.floor(z * inverse_resolution)),
                )
            )

        self.publish_cloud(message.header.stamp)
        rospy.loginfo_throttle(
            2.0,
            "Accumulated %d voxels (+%d from latest cloud)",
            len(self.voxels),
            len(self.voxels) - previous_count,
        )

    def publish_cloud(self, stamp):
        resolution = OUTPUT_RESOLUTION

        # Publish voxel centers. Clamp z at the filter boundaries so every
        # published point strictly remains inside [HEIGHT_MIN, HEIGHT_MAX].
        points = [
            (
                (ix + 0.5) * resolution,
                (iy + 0.5) * resolution,
                min(max((iz + 0.5) * resolution, HEIGHT_MIN), HEIGHT_MAX),
            )
            for ix, iy, iz in self.voxels
        ]
        header = Header(stamp=stamp, frame_id=self.frame_id)
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, points))


def main():
    rospy.init_node("occ_pc_accumulate", anonymous=False)
    PointCloudAccumulator()
    rospy.spin()


if __name__ == "__main__":
    main()
