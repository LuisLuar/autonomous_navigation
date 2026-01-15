#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class LaneTFToMapNode(Node):

    def __init__(self):
        super().__init__('lane_tf_to_map_node')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Sub
        self.create_subscription(
            Path,
            '/lane/tracked_path',
            self.tracked_cb,
            10
        )

        # Pub
        self.lane_ego_pub = self.create_publisher(
            Path,
            '/lane_ego',
            10
        )

        self.get_logger().info("Lane TF → Map node READY")

    def tracked_cb(self, msg: Path):

        if len(msg.poses) < 2:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF not available: {e}")
            return

        lane_ego = Path()
        lane_ego.header.frame_id = 'map'
        lane_ego.header.stamp = self.get_clock().now().to_msg()

        for pose in msg.poses:
            ps = PoseStamped()
            ps.header = lane_ego.header

            ps.pose = tf2_geometry_msgs.do_transform_pose(
                pose.pose,
                transform
            )

            lane_ego.poses.append(ps)

        self.lane_ego_pub.publish(lane_ego)


def main(args=None):
    rclpy.init(args=args)
    node = LaneTFToMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
