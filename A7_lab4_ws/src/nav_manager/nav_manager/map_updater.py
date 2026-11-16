"""Keeps the planning map up to date while driving and saves it for future runs.

AMCL keeps localizing on the map_server map (stable). This node loads the same map, refines it
from /scan using the AMCL pose, publishes the result on /map_live (the global costmap's static
layer listens to that), and writes it to `output_yaml` periodically and on shutdown.
A cell only changes after the lidar has disagreed with it for `persist_time` seconds, so people
walking by are ignored.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from nav_manager.map_update import PersistentMapUpdater, load_map, save_map, yaw_from_quat


class MapUpdater(Node):
    def __init__(self):
        super().__init__('map_updater')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('output_yaml', '')
        self.declare_parameter('update', True)
        self.declare_parameter('persist_time', 4.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('max_pose_std', 0.25)      # m: skip scans while AMCL is this unsure
        self.declare_parameter('max_yaw_rate', 0.6)       # rad/s: skip scans while turning fast
        self.declare_parameter('process_hz', 5.0)
        self.declare_parameter('publish_period', 2.0)
        self.declare_parameter('save_period', 30.0)

        p = self.get_parameter
        self.update_enabled = p('update').value
        self.output_yaml = p('output_yaml').value
        state, res, origin, meta = load_map(p('map_yaml').value)
        self.res, self.origin, self.meta = res, origin, meta
        self.up = PersistentMapUpdater(
            state, res, origin, persist_time=p('persist_time').value,
            min_range=p('min_range').value, max_range=p('max_range').value)
        self.max_pose_std = p('max_pose_std').value
        self.max_yaw_rate = p('max_yaw_rate').value
        self.min_dt = 1.0 / p('process_hz').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_std = float('inf')
        self.last_t = None
        self.last_yaw = None
        self.last_published_dirty = True

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map_live', latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(p('publish_period').value, self.publish_map)
        self.create_timer(p('save_period').value, self.save)
        self.publish_map()
        self.get_logger().info(
            'map_updater: %s, persist %.1fs, writing to %s' % (
                'UPDATING' if self.update_enabled else 'static (update:=false)',
                p('persist_time').value, self.output_yaml or '(not saving)'))

    def on_pose(self, msg):
        c = msg.pose.covariance
        self.pose_std = math.sqrt(max(c[0], c[7]))

    def on_scan(self, msg):
        if not self.update_enabled or self.pose_std > self.max_pose_std:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is not None and t - self.last_t < self.min_dt:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return
        q, tr = tf.transform.rotation, tf.transform.translation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        prev_t, prev_yaw = self.last_t, self.last_yaw
        self.last_t, self.last_yaw = t, yaw
        if prev_t is not None:
            dyaw = abs(math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw)))
            if dyaw / max(t - prev_t, 1e-3) > self.max_yaw_rate:
                return
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        ranges[(ranges < msg.range_min) | (ranges > msg.range_max)] = np.nan
        changed = self.up.integrate(ranges, angles, tr.x, tr.y, yaw, t)
        if changed:
            self.get_logger().info('map_updater: %d cells changed' % changed)

    def publish_map(self):
        if not self.last_published_dirty and not self.up.dirty:
            return
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = self.res
        m.info.width, m.info.height = self.up.w, self.up.h
        m.info.origin.position.x, m.info.origin.position.y = self.origin
        m.info.origin.orientation.w = 1.0
        m.data = self.up.to_occupancy_data().tolist()
        self.map_pub.publish(m)
        self.last_published_dirty = False

    def save(self):
        if not self.output_yaml or not self.up.dirty:
            return
        try:
            save_map(self.up.state, self.res, self.origin, self.output_yaml, self.meta)
            self.up.dirty = False
            self.last_published_dirty = True  # one more publish picks up the final state
            self.get_logger().info('map_updater: saved %s' % self.output_yaml)
        except Exception as e:
            self.get_logger().error('map_updater: save failed: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = MapUpdater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
