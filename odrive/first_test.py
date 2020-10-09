from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import turtle
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# tuning values no reduction
# pos = 275, vel = 0.000075 , int = 0 (good for holding torque)
# pos = 50, vel = 0.00035, int = 0.0001

# tuning values 20-79

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

# def animate(i, x, y, startTime):
#     print(f"i: {i}")
#     print(f"x: {x}")
#     print(f"y: {y}")
#     x.append(time.monotonic()-startTime)
#     y.append(odrv0.axis0.motor.current_control.Iq_measured)
#     ax1.set_xlim(0,time.monotonic())

#     ax1.clear()
#     ax1.plot(x,y)

# plt.ion()
# fig = plt.figure()
# plt.xlabel("time")
# plt.ylabel("measured current")
# ax1 = fig.add_subplot(111)
# ax1.set_ylim(-20,20)

# startTime = time.monotonic()

# x = [time.monotonic()-startTime]
# y = [odrv0.axis0.motor.current_control.Iq_measured]

# ani = animation.FuncAnimation(fig, animate, fargs=(x,y, startTime), interval=250)

# while True:
#     x.append(time.monotonic())
#     y.append(odrv0.axis0.motor.current_control.Iq_measured)
#     ax1.set_xlim(0,time.monotonic())

#     ax1.clear()
#     ax1.plot(x,y)

#     plt.draw()
#     plt.pause(0.1)
# plt.show(block=True)

def R3_increase():
    print("increasing R3 speed")
    odrv0.axis0.controller.vel_setpoint = 20000
    odrv0.axis1.controller.vel_setpoint = -20000
    
def R3_decrease():
    print("decreasing R3 speed")
    odrv0.axis0.controller.vel_setpoint = -20000
    odrv0.axis1.controller.vel_setpoint = 20000

def R3_nocrease():
    print("no R3 speed")
    odrv0.axis0.controller.vel_setpoint = 0
    odrv0.axis1.controller.vel_setpoint = 0

def T3_increase():
    print("increasing T3 speed")
    odrv0.axis0.controller.vel_setpoint = 20000
    odrv0.axis1.controller.vel_setpoint = 20000
    
def T3_decrease():
    print("decreasing T3 speed")
    odrv0.axis0.controller.vel_setpoint = -20000
    odrv0.axis1.controller.vel_setpoint = -20000

def T3_nocrease():
    print("no T3 speed")
    odrv0.axis0.controller.vel_setpoint = 0
    odrv0.axis1.controller.vel_setpoint = 0

tim = turtle.Turtle()
print("listening for arrow keys now....")
turtle.listen()

turtle.onkeypress(R3_increase, "Right")
turtle.onkeypress(R3_decrease, "Left")
turtle.onkeypress(T3_increase, "Up")
turtle.onkeypress(T3_decrease, "Down")

turtle.onkeyrelease(R3_nocrease, "Right")
turtle.onkeyrelease(R3_nocrease, "Left")
turtle.onkeyrelease(T3_nocrease, "Up")
turtle.onkeyrelease(T3_nocrease, "Down")

turtle.mainloop()




