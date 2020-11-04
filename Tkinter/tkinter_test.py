import tkinter as tk
from singleMotorControl import SingleMotor

SPEEDS = [1,2,4,6,8,10,20]

def updateSpeed():
    speedLabel["text"]=f"speed is {speedVariable.get()} rad/s"

def getSpeed():
    return speedVariable.get()

window = tk.Tk()

speedFrame = tk.Frame()
speedFrame.rowconfigure(0, minsize=100, weight=1)
speedFrame.columnconfigure([0, 1, 2], minsize=100, weight=1)
speedLabel = tk.Label(master=speedFrame, text="speed (rad/s)")
speedLabel.grid(row=0, column=0)

speedVariable = tk.StringVar(speedFrame)
speedVariable.set(SPEEDS[0])
speedDropdown = tk.OptionMenu(speedFrame, speedVariable, *SPEEDS)
speedDropdown.grid(row=0, column=1)

speedConfirm = tk.Button(master=speedFrame, text="ok", command=updateSpeed)
speedConfirm.grid(row=0,column=2)

speedFrame.pack()

motorR1 = SingleMotor(window, getSpeed, "motorR1", 'q', 'a')
motorT1 = SingleMotor(window, getSpeed, "motorT1", 'w', 's')
motorT2 = SingleMotor(window, getSpeed, "motorT2", 'e', 'd')
motorR2 = SingleMotor(window, getSpeed, "motorR2", 'r', 'f')
motorT3 = SingleMotor(window, getSpeed, "motorT3", 't', 'g')
motorR3 = SingleMotor(window, getSpeed, "motorT2", 'y', 'h')

window.mainloop()