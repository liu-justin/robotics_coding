#!c:\users\justi\appdata\local\programs\python\python36-32\python.exe
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import turtle

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
odrv0 = odrive.find_any()

# Calibrate motor and wait for it to finish
print("starting calibration...")
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis1.current_state != AXIS_STATE_IDLE and odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

# To read a value, simply read the property
print("Bus voltage is " + str(odrv0.vbus_voltage) + "V")

# Or to change a value, just assign to the property
odrv0.axis1.controller.pos_setpoint = 3.14
print("Position setpoint is " + str(odrv0.axis1.controller.pos_setpoint))

# And this is how function calls are done:
for i in [1,2,3,4]:
    print('voltage on GPIO{} is {} Volt'.format(i, odrv0.get_adc_voltage(i)))

# # A sine wave to test
# t0 = time.monotonic()
# while True:
#     setpoint = 10000.0 * math.sin((time.monotonic() - t0)*2)
#     print("goto " + str(int(setpoint)))
#     odrv0.axis1.controller.pos_setpoint = setpoint
#     time.sleep(0.01)

def increase():
    print("increasing speed")
    odrv0.axis0.controller.vel_setpoint = 10000
    
def decrease():
    print("decreasing speed")
    odrv0.axis0.controller.vel_setpoint = -10000

def nocrease():
    print("no speed")
    odrv0.axis0.controller.vel_setpoint = 0

tim = turtle.Turtle()
print("listening for arrow keys now....")
turtle.listen()

turtle.onkeypress(increase, "Right")
turtle.onkeypress(decrease, "Left")
turtle.onkeyrelease(nocrease, "Right")
turtle.onkeyrelease(nocrease, "Left")

turtle.mainloop()




