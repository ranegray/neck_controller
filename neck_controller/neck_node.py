#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import dynamixel_sdk as dxl

# TODO: Parameterize these values
YAW_NEUTRAL_OFFSET = 1.5
PITCH_NEUTRAL_OFFSET = 3.2

YAW_MIN = math.radians(-70)   # Left - 1.22
YAW_MAX = math.radians(70)    # Right + 1.22
PITCH_MIN = math.radians(-35)    # Down - 0.61
PITCH_MAX = math.radians(65)     # Up + 1.14

ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108

PROFILE_VELOCITY = 50     # 0-1023 (experiment with values, lower = slower)
PROFILE_ACCELERATION = 10 # 0-32767 (experiment, lower = smoother)

class DynamixelHandler:
    def __init__(self, port='/dev/ttyUSB0', baud=57600, ids={'yaw': 0, 'pitch': 1}):
        self.ids = ids
        self.portHandler = dxl.PortHandler(port)
        self.packetHandler = dxl.PacketHandler(2.0)
        if not self.portHandler.openPort():
            raise RuntimeError("Failed to open port")
        
        if not self.portHandler.setBaudRate(baud):
            raise RuntimeError("Failed to set baudrate")
        
        # Set velocity and acceleration profile for each joint
        for dxl_id in self.ids.values():
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)

        # Enable torque for each joint
        ADDR_TORQUE_ENABLE = 64
        for dxl_id in self.ids.values():
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        self.joint_pos = {'yaw': 0.0, 'pitch': 0.0}

    def radians_to_dxl(self, rad):
        # XL330: 0–4095 maps to 0–360°
        return int((rad % (2 * 3.14159)) * 4095 / (2 * 3.14159))

    def dxl_to_radians(self, pos):
        return float(pos) * (2 * 3.14159) / 4095

    def set_joint(self, joint_name, rad):
        dxl_pos = self.radians_to_dxl(rad)
        dxl_id = self.ids[joint_name]
        ADDR_GOAL_POSITION = 116
        self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, dxl_pos)
        self.joint_pos[joint_name] = rad

    def get_joint(self, joint_name):
        dxl_id = self.ids[joint_name]
        ADDR_PRESENT_POSITION = 132
        dxl_pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
        return self.dxl_to_radians(dxl_pos)
    
    def disable_torque(self):
        ADDR_TORQUE_ENABLE = 64
        for dxl_id in self.ids.values():
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)

class NeckJointController(Node):
    def __init__(self):
        super().__init__('neck_joint_controller')
        # --- Dynamixel setup
        # self.dxl = DynamixelHandler('/dev/ttyUSB0', 57600)
        self.dxl = DynamixelHandler('/dev/ttyUSB0', 57600, {'yaw': 0, 'pitch': 1})
        self.joint_names = ['yaw', 'pitch']

        # --- Subscriber for command
        self.sub_cmd = self.create_subscription(
            # TODO: Use joint_state msgs instead of Float64MultiArray
            Float64MultiArray,
            '/neck_controller/command',
            self.command_callback,
            10
        )
        # --- Publisher for state
        self.pub_state = self.create_publisher(
            # TODO: Use joint_state msgs instead of Float64MultiArray
            Float64MultiArray,
            '/neck_controller/state',
            10
        )
        # --- Timer to publish state at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_state)
        self.last_command = [0.0, 0.0]
        self.get_logger().info("NeckJointController ready.")

    def command_callback(self, msg):
        desired_yaw, desired_pitch = msg.data

        clamped_yaw = max(min(desired_yaw, YAW_MAX), YAW_MIN)
        clamped_pitch = max(min(desired_pitch, PITCH_MAX), PITCH_MIN)

        self.get_logger().info(f"Clamped logical yaw, pitch: {clamped_yaw}, {clamped_pitch}")

        yaw_cmd = clamped_yaw + YAW_NEUTRAL_OFFSET
        pitch_cmd = clamped_pitch + PITCH_NEUTRAL_OFFSET

        self.dxl.set_joint('yaw', yaw_cmd)
        self.dxl.set_joint('pitch', pitch_cmd)
        self.last_command = [clamped_yaw, clamped_pitch]

    def publish_state(self):
        yaw_state = self.dxl.get_joint('yaw') - YAW_NEUTRAL_OFFSET
        pitch_state = self.dxl.get_joint('pitch') - PITCH_NEUTRAL_OFFSET
        msg = Float64MultiArray()
        msg.data = [yaw_state, pitch_state]
        self.pub_state.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NeckJointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.dxl.disable_torque()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
