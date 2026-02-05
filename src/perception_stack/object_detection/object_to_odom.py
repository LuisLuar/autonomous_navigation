#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from custom_interfaces.msg import ObjectInfoArray
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

class ObjectToOdomNode(Node):

    def __init__(self):
        super().__init__('object_to_odom_node')

        # ================= SUB =================
        self.create_subscription(
            ObjectInfoArray,
            '/objects/fused_info',
            self.objects_callback,
            10
        )

        # ================= PUB =================
        self.pub_points = self.create_publisher(
            PoseArray,
            '/objects/points_odom',
            10
        )

        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/objects/markers',
            10
        )

    # =====================================================
    def objects_callback(self, msg: ObjectInfoArray):
        poses = PoseArray()
        poses.header.stamp = msg.header.stamp
        poses.header.frame_id = "base_footprint"

        markers = MarkerArray()

        # ===== LIMPIAR MARKERS =====
        clear = Marker()
        clear.header.frame_id = "base_footprint"
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, obj in enumerate(msg.objects):

            if not obj.distance_valid:
                continue

            pose = Pose()
            pose.position.x = float(obj.distance)
            pose.position.y = float(obj.lateral_offset)
            pose.position.z = 0.0
            pose.orientation.w = 1.0

            poses.poses.append(pose)

            marker = Marker()
            marker.header = poses.header
            marker.ns = "objects"   # IMPORTANTE
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose = pose
            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 1.7

            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.1
            marker.color.a = 0.9

            markers.markers.append(marker)

        self.pub_points.publish(poses)
        self.pub_markers.publish(markers)

def main():
    rclpy.init()
    node = ObjectToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()