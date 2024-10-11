#!/usr/bin/env python3

# -------------------- Import Packages --------------------
import rclpy
from rclpy.node import Node
# from std_msgs.msg import Int64, Bool
import dynamixel_sdk as dxl
from motor_control.msg import RRPkinematics

## -------------------- Dynamixel Configuration --------------------
# Control table address for Dynamixel
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION   = 116
# Torque on/off
TORQUE_ENABLE = 1        # Value for enabling torque
TORQUE_DISABLE = 0       # Value for disabling torque
# Protocol 2.0
PROTOCOL_VERSION        = 2.0
# Default setting (use appropriate values)
BAUDRATE                = 57600
DEVICENAME              = '/dev/ttyUSB0'  # Change to COM3 if on Windows
# Motor IDs
ROLL_MOTOR = 11
PITCH_MOTOR = 12
EXTENSION_MOTOR = 13
# Reference/home position for zeroing
ROLL_HOME_POS = 500
PITCH_HOME_POS = 3855

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
        
        self.set_extended_position_mode(ROLL_MOTOR)
        self.set_extended_position_mode(PITCH_MOTOR)
        
        self.set_profile_velocity(ROLL_MOTOR, 100)
        self.set_profile_velocity(PITCH_MOTOR, 100)
        
        self.enable_motor(ROLL_MOTOR)
        self.enable_motor(PITCH_MOTOR)
        self.enable_motor(EXTENSION_MOTOR)
        
        self.write_position(ROLL_MOTOR, ROLL_HOME_POS)
        self.write_position(PITCH_MOTOR, PITCH_HOME_POS)
        
    def follower_callback(self, msg: RRPkinematics) -> None:
        self.write_position(ROLL_MOTOR, msg.roll_angle + ROLL_HOME_POS)
        self.write_position(PITCH_MOTOR, msg.pitch_angle + PITCH_HOME_POS)
        # self.write_position(EXTENSION_MOTOR, msg.extension_angle)
    
    def set_extended_position_mode(self, motor_id) -> None:
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_OPERATING_MODE, 4)
        if result != dxl.COMM_SUCCESS:
            self.get_logger().error(f"Error setting extended position mode for motor {motor_id}")
        elif error != 0:
            print(f"Dynamixel error: {self.packet_handler.getRxPacketError(error)}")
        else:
            print(f"Set motor {motor_id} to extended position mode!")
            
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

    def set_profile_velocity(self, motor_id, profile_velocity) -> None:
        result, error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_PROFILE_VELOCITY, profile_velocity)
        # ADD ERROR HANDLING HERE
        
    def enable_motor(self, motor_id) -> None:
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if result != dxl.COMM_SUCCESS:
            print(f"Error enabling torque: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            print(f"Dynamixel error: {self.packet_handler.getRxPacketError(error)}")
        else:
            print("Torque enabled")
            
    def disable_motor(self, motor_id) -> None:
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if result != dxl.COMM_SUCCESS:
            print(f"Error disabling torque: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            print(f"Dynamixel error: {self.packet_handler.getRxPacketError(error)}")
        else:
            print("Torque disabled")
            
    def close_port(self):
        # Disable motors and close the port when done
        self.disable_motor(ROLL_MOTOR)
        self.disable_motor(PITCH_MOTOR)
        self.disable_motor(EXTENSION_MOTOR)
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
    
