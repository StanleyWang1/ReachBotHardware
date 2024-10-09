#!/usr/bin/env python3

# -------------------- Import Packages --------------------
import rclpy
from rclpy.node import Node
# from std_msgs.msg import Int64, Bool
import dynamixel_sdk as dxl
from motor_control.msg import RRPkinematics

## -------------------- Dynamixel Configuration --------------------
# Control table address for Dynamixel
ADDR_PRESENT_POSITION   = 132
# Protocol 2.0
PROTOCOL_VERSION        = 2.0
# Default setting (use appropriate values)
BAUDRATE                = 57600
DEVICENAME              = '/dev/ttyUSB0'  # Change to COM3 if on Windows
# Motor IDs
ROLL_MOTOR = 1
PITCH_MOTOR = 2
EXTENSION_MOTOR = 3

class LeaderRobot(Node):
    def __init__(self) -> None:
        super().__init__("LeaderRobot")
        self.leader_pub = self.create_publisher(RRPkinematics, "/LeaderKinematics", 10)
        self.leader_timer = self.create_timer(0.1, self.leader_callback)

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
        
    def leader_callback(self) -> None:
        msg = RRPkinematics()
        msg.roll_angle = self.read_position(ROLL_MOTOR)
        msg.pitch_angle = self.read_position(PITCH_MOTOR)
        msg.extension_angle = self.read_position(EXTENSION_MOTOR)
        self.leader_pub.publish(msg)
    
    def read_position(self, motor_id):
        # Read present position of the Dynamixel motor
        position, result, error = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, ADDR_PRESENT_POSITION)
        if result != dxl.COMM_SUCCESS:
            self.get_logger().error(f"Failed to read position from motor {motor_id}: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            self.get_logger().error(f"Motor {motor_id} error: {self.packet_handler.getRxPacketError(error)}")
        else:
            self.get_logger().info(f"Motor {motor_id} position: {position}")
        return position

    def close_port(self):
        # Disable motors and close the port when done
        # self.disable_motor(ROLL_MOTOR)
        # self.disable_motor(PITCH_MOTOR)
        # self.disable_motor(EXTENSION_MOTOR)
        self.port_handler.closePort()
        
if __name__ == "__main__":
    rclpy.init()
    node = LeaderRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_port()
        rclpy.shutdown()
    