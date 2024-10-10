#!/usr/bin/env python3

# -------------------- Import Packages --------------------
import rclpy
from rclpy.node import Node
# from std_msgs.msg import Int64, Bool
import dynamixel_sdk as dxl
from motor_control.msg import RRPkinematics

## -------------------- Dynamixel Configuration --------------------
# Control table address for Dynamixel
ADDR_GOAL_POSITION   = 116
# Protocol 2.0
PROTOCOL_VERSION        = 2.0
# Default setting (use appropriate values)
BAUDRATE                = 57600
DEVICENAME              = '/dev/ttyUSB0'  # Change to COM3 if on Windows
# Motor IDs
ROLL_MOTOR = 4
PITCH_MOTOR = 5
EXTENSION_MOTOR = 6

class FollowerRobot(Node):
    def __init__(self) -> None:
        super().__init__("FollowerRobot")
        self.motor_sub = self.create_subscription(RRPkinematics, "/LeaderKinematics", self.follower_callback, 10)

        # Initialize PortHandler and PacketHandler
        self.port_handler = dxl.PortHandler(DEVICENAME)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
        # Open the port
        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open the port")
            return
        # Set baudrate
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            return
        
    def follower_callback(self, msg: RRPkinematics) -> None:
        self.write_position(ROLL_MOTOR, msg.roll_angle)
        self.write_position(PITCH_MOTOR, msg.pitch_angle)
        self.write_position(EXTENSION_MOTOR, msg.extension_angle)
    
    def write_position(self, motor_id, position):
        # Read present position of the Dynamixel motor
        result, error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_GOAL_POSITION, position)
        if result != dxl.COMM_SUCCESS:
            self.get_logger().error(f"Failed to write position from motor {motor_id}: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            self.get_logger().error(f"Motor {motor_id} error: {self.packet_handler.getRxPacketError(error)}")
        else:
            self.get_logger().info(f"Set Motor {motor_id} to position: {position}")
        return position

    def close_port(self):
        # Disable motors and close the port when done
        # self.disable_motor(ROLL_MOTOR)
        # self.disable_motor(PITCH_MOTOR)
        # self.disable_motor(EXTENSION_MOTOR)
        self.port_handler.closePort()
        
if __name__ == "__main__":
    rclpy.init()
    node = FollowerRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_port()
        rclpy.shutdown()
    
