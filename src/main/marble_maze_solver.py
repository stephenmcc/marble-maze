
import os
import time

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

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID1                     = 10                # Dynamixel ID1 : 10
DXL_ID2                     = 20                # Dynamixel ID1 : 20
BAUDRATE                    = 1000000
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

cwAngleLimit1, cwResult, cwAngleError = packetHandler.read2ByteTxRx(portHandler, DXL_ID1, 6)
print("cwAngleLimit1")
print(cwAngleLimit1)

cwAngleLimit2, cwResult, cwAngleError = packetHandler.read2ByteTxRx(portHandler, DXL_ID2, 6)
print("cwAngleLimit2")
print(cwAngleLimit2)

ccwAngleLimit1, ccwResult, ccwAngleError = packetHandler.read2ByteTxRx(portHandler, DXL_ID1, 8)
print("ccwAngleLimit1")
print(ccwAngleLimit1)

ccwAngleLimit2, ccwResult, ccwAngleError = packetHandler.read2ByteTxRx(portHandler, DXL_ID2, 8)
print("ccwAngleLimit2")
print(ccwAngleLimit2)

goalPosition1, goalResult, goalError = packetHandler.read2ByteTxRx(portHandler, DXL_ID1, 30)
print("goalPosition1")
print(goalPosition1)

goalPosition2, goalResult, goalError = packetHandler.read2ByteTxRx(portHandler, DXL_ID2, 30)
print("goalPosition2")
print(goalPosition2)

# Enable Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

time.sleep(1)

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_MX_GOAL_POSITION, 450)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_MX_GOAL_POSITION, 450)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

time.sleep(2)

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_MX_GOAL_POSITION, 550)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_MX_GOAL_POSITION, 550)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

time.sleep(2)

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_MX_GOAL_POSITION, 500)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_MX_GOAL_POSITION, 500)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

time.sleep(1)

# Disable Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
