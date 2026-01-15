#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class DroidCamGStreamer(Node):

    def __init__(self):
        super().__init__('droidcam_gstreamer_node')

        self.pub = self.create_publisher(
            CompressedImage,
            '/camera/droidcam/image_raw/compressed',
            10
        )

        Gst.init(None)

        pipeline_str = (
            "v4l2src device=/dev/video0 ! "
            "video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! "
            "jpegenc quality=80 ! "
            "image/jpeg ! "
            "appsink name=sink emit-signals=true max-buffers=1 drop=true"
        )


        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.connect("new-sample", self.on_new_sample)

        self.pipeline.set_state(Gst.State.PLAYING)

        self.get_logger().info("DroidCam GStreamer iniciado (30 FPS)")

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buffer = sample.get_buffer()

        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"   # 🔴 CLAVE
        msg.format = "jpeg"
        msg.data = map_info.data

        self.pub.publish(msg)

        buffer.unmap(map_info)
        return Gst.FlowReturn.OK


def main():
    rclpy.init()
    node = DroidCamGStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
