#!/usr/bin/env python

from v1_two_axis.srv import *
from v1_two_axis.msg import *
import rospy

import stepFinder as s
import math
import numpy as np
import time

# send msg with only velocity; have 6 different topics for 6 motors

def server():
    rospy.init_node('server')
    pub0 = rospy.Publisher('chatter_motor0', velocity,queue_size=1)
    pub1 = rospy.Publisher('chatter_motor1', velocity,queue_size=1)
    pub2 = rospy.Publisher('chatter_motor2', velocity,queue_size=1)
    pub3 = rospy.Publisher('chatter_motor3', velocity,queue_size=1)
    pub4 = rospy.Publisher('chatter_motor4', velocity,queue_size=1)
    pub5 = rospy.Publisher('chatter_motor5', velocity,queue_size=1)

    value = int(lbl_value["text"])
    lbl_value["text"] = f"{value - 1}"

    window = tk.Tk()

    window.rowconfigure(0, minsize=50, weight=1)
    window.columnconfigure([0, 1, 2], minsize=50, weight=1)

    btn_decrease = tk.Button(master=window, text="-", command=decrease)
    btn_decrease.grid(row=0, column=0, sticky="nsew")

    lbl_value = tk.Label(master=window, text="0")
    lbl_value.grid(row=0, column=1)

    btn_increase = tk.Button(master=window, text="+", command=increase)
    btn_increase.grid(row=0, column=2, sticky="nsew")

    window.mainloop()

if __name__ == "__main__":
import tkinter as tk

def increase():
    value = int(lbl_value["text"])
    lbl_value["text"] = f"{value + 1}"

def decrease():

    server()