import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
import json


class RelayTrajNode(Node):
    def __init__(self):
        super().__init__('relay_traj_node')

        # Subscribe to computed trajectories
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/compute_traj/trajectory',
            self.trajectory_callback,
            10
        )

        # Subscribe to previous joint_values to cache is_gripped and is_on
        self.joint_values_sub = self.create_subscription(
            String,
            '/joint_values',
            self.joint_values_callback,
            10
        )

        # Publisher for new joint values
        self.publisher = self.create_publisher(String, '/joint_values', 10)

        # Cached toggle states
        self.cached_is_gripped = False
        self.cached_is_on = False

        self.get_logger().info("relay_traj_node ready and listening.")

    def joint_values_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if 'is_gripped' in data:
                self.cached_is_gripped = data['is_gripped']
            if 'is_on' in data:
                self.cached_is_on = data['is_on']
        except Exception as e:
            self.get_logger().warn(f"Failed to parse joint_values: {e}")

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn("Received empty trajectory.")
            return

        final_point = msg.points[-1]
        joint_data = {name: pos for name, pos in zip(msg.joint_names, final_point.positions)}

        # Include cached state
        joint_data['is_gripped'] = self.cached_is_gripped
        # joint_data['is_on'] = True
        joint_data['is_on'] = self.cached_is_on

        json_msg = String()
        json_msg.data = json.dumps(joint_data)
        self.publisher.publish(json_msg)

        self.get_logger().info("Published last trajectory point with cached toggle states.")


def main(args=None):
    rclpy.init(args=args)
    node = RelayTrajNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
