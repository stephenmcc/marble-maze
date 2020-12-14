import os
import time

from math_utils import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control tables addresses here:
# https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table-data-address

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_CW_LIMIT           = 6
ADDR_MX_CCW_LIMIT          = 8

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 1000000
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

LEVEL_ID_10                 = 625
LEVEL_ID_20                 = 670

MAX_CONTROL_INPUT = 115

class MotorDriver:
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    def __init__(self, motor_id, level_position):
        self.motor_id = motor_id
        self.level_position = level_position

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def create_motor_x():
        return MotorDriver(10, LEVEL_ID_10)

    def create_motor_y():
        return MotorDriver(20, LEVEL_ID_20)

    def print_current_state(self):
        cwAngleLimit, cwResult, cwAngleError = self.packetHandler.read2ByteTxRx(self.portHandler, self.motor_id, ADDR_MX_CW_LIMIT)
        print("CW Angle Limit: " + str(cwAngleLimit))

        ccwAngleLimit, ccwResult, ccwAngleError = self.packetHandler.read2ByteTxRx(self.portHandler, self.motor_id, ADDR_MX_CCW_LIMIT)
        print("CCW Angle Limit: " + str(ccwAngleLimit))

        goalPosition, goalResult, goalError = self.packetHandler.read2ByteTxRx(self.portHandler, self.motor_id, ADDR_MX_GOAL_POSITION)
        print("Goal position: " + str(goalPosition))

    def enable_torque(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.motor_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def disable_torque(self):
        # Disable Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.motor_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def set_goal_position(self, goal_position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.motor_id, ADDR_MX_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def go_to_level(self):
        self.set_goal_position(self.level_position)

    def set_goal_relative_to_level(self, delta_level):
        clamped_input = clamp(delta_level, MAX_CONTROL_INPUT)
        self.set_goal_position(self.level_position + clamped_input)

    def shutdown(self):
        self.portHandler.closePort()
