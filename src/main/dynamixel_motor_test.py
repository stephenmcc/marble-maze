import time
from dynamixel_motor_driver import MotorDriver

motorDriver1 = MotorDriver(10)
motorDriver2 = MotorDriver(20)

motorDriver1.print_current_state()
motorDriver2.print_current_state()

time.sleep(1)

motorDriver1.shutdown()
motorDriver2.shutdown()
