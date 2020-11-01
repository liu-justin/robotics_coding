import tkinter as tk

window = tk.Tk()

class singleMotorControl:
    def __init__(self, name):
        self.name = name
        self.frame = tk.Frame()

        self.frame.rowconfigure(0, minsize=100, weight=1)
        self.frame.columnconfigure([0, 1, 2, 3], minsize=100, weight=1)

        self.name_label = tk.Label(master=self.frame, text=f"{self.name}")
        self.name_label.grid(row=0, column=0)

        self.btn_decrease = tk.Button(master=self.frame, text="-", command=self.decrease)
        self.btn_decrease.grid(row=0, column=1, sticky="nsew")

        self.label = tk.Label(master=self.frame, text="0")
        self.label.grid(row=0, column=2)

        self.btn_increase = tk.Button(master=self.frame, text="+", command=self.increase)
        self.btn_increase.grid(row=0, column=3, sticky="nsew")

        self.window.bind('<KeyPress>', self.printKey)
        self.window.bind('<Key-a>', self.decrease)

        self.frame.pack()

    def printKey(self, e):
        print(f"keypressed: {e}")

    def increase(self):
        value = int(self.label["text"])
        self.label["text"] = f"{value + 1}"

    def decrease(self):
        value = int(self.label["text"])
        self.label["text"] = f"{value - 1}"

SPEEDS = [1,2,4,6,8,10,20]

def updateSpeed():
    speedLabel["text"]=f"speed is {speedVariable.get()} rad/s"

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

motorR1 = singleMotorControl("motorR1")
motorT1 = singleMotorControl("motorT1")

window.mainloop()