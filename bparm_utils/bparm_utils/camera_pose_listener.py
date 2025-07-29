#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class CameraPosePublisher(Node):
    def __init__(self):
        super().__init__('camera_pose_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.pose_pub = self.create_publisher(PoseStamped, '/camera_pose', 10)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                'linkCamera',
                now
            )

            pose = PoseStamped()
            pose.header.stamp = trans.header.stamp
            pose.header.frame_id = trans.header.frame_id

            # 🛠️ Use correct types (Point and Quaternion)
            pose.pose.position = Point(
                x=trans.transform.translation.x,
                y=trans.transform.translation.y,
                z=trans.transform.translation.z
            )

            pose.pose.orientation = Quaternion(
                x=trans.transform.rotation.x,
                y=trans.transform.rotation.y,
                z=trans.transform.rotation.z,
                w=trans.transform.rotation.w
            )

            self.pose_pub.publish(pose)

            # self.get_logger().info(
            #     f"Camera pose wrt base_link:\n"
            #     f"  position = (x={pose.pose.position.x:.3f}, "
            #     f"y={pose.pose.position.y:.3f}, "
            #     f"z={pose.pose.position.z:.3f})\n"
            #     f"  orientation = (x={pose.pose.orientation.x:.3f}, "
            #     f"y={pose.pose.orientation.y:.3f}, "
            #     f"z={pose.pose.orientation.z:.3f}, "
            #     f"w={pose.pose.orientation.w:.3f})"
            # )

        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
