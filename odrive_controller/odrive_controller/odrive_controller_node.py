import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import can
import struct
import math
from gpiozero import AngularServo
from time import sleep

# === Constants ===
CMD_SET_AXIS_STATE = 0x07
CMD_SET_INPUT_MODE = 0x0B
CMD_SET_INPUT_POS = 0x0C

AXIS_STATE_CLOSED_LOOP_CONTROL = 8
AXIS_STATE_IDLE = 1

INPUT_MODE_POS_FILTER = 0x03
CONTROL_MODE_POSITION = 0x03

CAN_CHANNEL = 'can0'
CAN_BITRATE = 250000

# Scaling factor: 1 rad = 100 / (2*pi) encoder counts
SCALE = 100 / (2 * math.pi)

# Hardcoded node ID mapping
ODRIVE_NODE_IDS = [0, 2, 3, 4, 5, 6]

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller_node')
        self.bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan', bitrate=CAN_BITRATE)

        # Setup servo on GPIO14
        self.servo = AngularServo(14, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        self.set_servo_angle(0)  # Initialize servo at 0°

        self.subscription = self.create_subscription(
            String,
            'joint_values',
            self.listener_callback,
            10
        )

    def init_odrives(self):
        for node_id in ODRIVE_NODE_IDS:
            self.send_set_input_mode(node_id, INPUT_MODE_POS_FILTER, CONTROL_MODE_POSITION)
        self.get_logger().info("Set input modes")

        for node_id in ODRIVE_NODE_IDS:
            self.send_set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
        self.get_logger().info("Entered closed-loop control")

    def shutdown_odrives(self):
        for node_id in ODRIVE_NODE_IDS:
            self.send_set_axis_state(node_id, AXIS_STATE_IDLE)
        self.get_logger().info("Exited to IDLE state")

    def send_set_axis_state(self, node_id, state):
        arb_id = (node_id << 5) | CMD_SET_AXIS_STATE
        data = struct.pack('<I', state)
        self.bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))

    def send_set_input_mode(self, node_id, input_mode, control_mode):
        arb_id = (node_id << 5) | CMD_SET_INPUT_MODE
        data = struct.pack('<II', control_mode, input_mode)
        self.bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))

    def send_set_input_pos(self, node_id, pos, vel_ff=0, torque_ff=0):
        arb_id = (node_id << 5) | CMD_SET_INPUT_POS
        scaled_pos = pos * SCALE
        data = struct.pack('<fhh', scaled_pos, vel_ff, torque_ff)
        self.bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))

    def set_servo_angle(self, angle):
        try:
            self.servo.angle = angle
        except Exception as e:
            self.get_logger().error(f"Failed to set servo angle: {e}")

    def listener_callback(self, msg):
        try:
            values = json.loads(msg.data)
            is_on = values.get('is_on', False)

            if is_on:
                # Initialize ODrives if not already initialized
                if not hasattr(self, 'odrives_initialized') or not self.odrives_initialized:
                    self.init_odrives()
                    self.odrives_initialized = True

                # Send position commands only when ON
                if 'joint1' in values:
                    self.send_set_input_pos(0, -values['joint1'])
                if 'joint4' in values:
                    self.send_set_input_pos(2, values['joint4'])
                if 'joint5' in values:
                    self.send_set_input_pos(3, -values['joint5'])
                if 'joint6' in values:
                    self.send_set_input_pos(4, -values['joint6'])

                if 'joint2' in values and 'joint3' in values:
                    pitch = values['joint2']
                    yaw = values['joint3']
                    self.send_set_input_pos(5, yaw + pitch)
                    self.send_set_input_pos(6, yaw - pitch)
            else:
                # Shutdown ODrives only once if currently initialized
                if hasattr(self, 'odrives_initialized') and self.odrives_initialized:
                    self.shutdown_odrives()
                    self.odrives_initialized = False

            # Handle servo angle regardless of is_on
            is_gripped = values.get('is_gripped', False)
            if is_gripped:
                self.set_servo_angle(70)
            else:
                self.set_servo_angle(0)

        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

def main():
    rclpy.init()
    node = ODriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down")
    finally:
        node.shutdown_odrives()
        node.bus.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
