#! /usr/bin/python3
import math

import rclpy
from rclpy.node import Node
from tf2_ros import TFMessage, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):

    def __init__(self):
        super().__init__('robot_ee_listener')
        self.declare_parameter("target_frame", "world")
        self.target_frame = self.get_parameter('target_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(TFMessage, '/current_pose', 1)
        self.timer = self.create_timer(0.01, self.on_timer)

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'arm0_grasp_point'

        try:
            t = self.tf_buffer.lookup_transform(
                from_frame_rel,
                to_frame_rel,
                rclpy.time.Time())
            msg = TFMessage()
            msg.transforms.append(t)
            self.publisher.publish(msg)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
