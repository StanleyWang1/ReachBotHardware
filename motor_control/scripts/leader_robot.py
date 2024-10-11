#!/usr/bin/env python3

# -------------------- Import Packages --------------------
import rclpy
from rclpy.node import Node
# from std_msgs.msg import Int64, Bool
import dynamixel_sdk as dxl
from motor_control.msg import RRPkinematics

## -------------------- Dynamixel Configuration --------------------
# Control table address for Dynamixel
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION   = 132
# Torque on/off
TORQUE_ENABLE = 1        # Value for enabling torque
TORQUE_DISABLE = 0       # Value for disabling torque
# Protocol 2.0
PROTOCOL_VERSION        = 2.0
# Connection settings (use appropriate values)
BAUDRATE                = 57600
DEVICENAME              = '/dev/ttyUSB0'  # Change to COM3 if on Windows
# Motor IDs
ROLL_MOTOR = 1
PITCH_MOTOR = 2
EXTENSION_MOTOR = 3
# Reference/home position for zeroing
ROLL_HOME_POS = 2085
PITCH_HOME_POS = 2030

class LeaderRobot(Node):
    def __init__(self) -> None:
        super().__init__("LeaderRobot")
        self.leader_pub = self.create_publisher(RRPkinematics, "/LeaderKinematics", 10)
        self.leader_timer = self.create_timer(0.05, self.leader_callback)

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
        msg.roll_angle = self.read_position(ROLL_MOTOR) - ROLL_HOME_POS
        msg.pitch_angle = self.read_position(PITCH_MOTOR) - PITCH_HOME_POS
        msg.extension_angle = self.read_position(EXTENSION_MOTOR)
        self.leader_pub.publish(msg)
    
    def read_position(self, motor_id) -> None:
        # Read present position of the Dynamixel motor
        position, result, error = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, ADDR_PRESENT_POSITION)
        if result != dxl.COMM_SUCCESS:
            self.get_logger().error(f"Failed to read position from motor {motor_id}: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            self.get_logger().error(f"Motor {motor_id} error: {self.packet_handler.getRxPacketError(error)}")
        else:
            self.get_logger().info(f"Motor {motor_id} position: {position}")
        return position
    
    def enable_motor(self, motor_id) -> None:
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if result != dxl.COMM_SUCCESS:
            print(f"Error enabling torque: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"Dynamixel error: {self.packetHandler.getRxPacketError(error)}")
        else:
            print("Torque enabled")
            
    def disable_motor(self, motor_id) -> None:
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if result != dxl.COMM_SUCCESS:
            print(f"Error disabling torque: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"Dynamixel error: {self.packetHandler.getRxPacketError(error)}")
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
    node = LeaderRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_port()
        rclpy.shutdown()
    