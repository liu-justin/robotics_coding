import tkinter as tk
from debouncer import Debouncer

class SingleMotor(tk.Frame):
    def __init__(self, master, getSpeed, name, increaseKey, decreaseKey):
        super().__init__(master)
        self.name = name
        self.frame = tk.Frame()

        self.getSpeed = getSpeed

        self.frame.rowconfigure(0, minsize=100, weight=1)
        self.frame.columnconfigure([0, 1, 2, 3], minsize=100, weight=1)

        self.name_label = tk.Label(master=self.frame, text=f"{self.name}")
        self.name_label.grid(row=0, column=0)

        self.btn_decrease = tk.Button(master=self.frame, text="-", command=self.increase)
        self.btn_decrease.bind('<Button-1>', self.decrease)
        self.btn_decrease.grid(row=0, column=1, sticky="nsew")

        self.label = tk.Label(master=self.frame, text="0")
        self.label.grid(row=0, column=2)

        self.btn_increase = tk.Button(master=self.frame, text="+", command=self.decrease) # command=self.stop for debouncer fail
        self.btn_increase.bind('<Button-1>', self.increase)
        self.btn_increase.grid(row=0, column=3, sticky="nsew")

        # failsafe for when debouncer fails
        # master.bind('<KeyPress-' + increaseKey + '>', self.increase)
        # master.bind('<KeyRelease-' + increaseKey + '>', self.stop)
        # master.bind('<KeyPress-' + decreaseKey + '>', self.decrease)
        # master.bind('<KeyRelease-' + decreaseKey + '>', self.stop)

        self.up_debouncer = Debouncer(self.increase, self.decrease)
        master.bind('<KeyPress-' + increaseKey + '>', self.up_debouncer.pressed)
        master.bind('<KeyRelease-' + increaseKey + '>', self.up_debouncer.released)
        self.down_debouncer = Debouncer(self.decrease, self.increase)
        master.bind('<KeyPress-' + decreaseKey + '>', self.down_debouncer.pressed)
        master.bind('<KeyRelease-' + decreaseKey + '>', self.down_debouncer.released)

        self.frame.pack()

    def keyPress(self, e):
        print(f"keypressed: {e}")

    def increase(self, e=None):
        value = int(self.label["text"])
        self.label["text"] = f"{value+int(self.getSpeed())}"
        print("up")

    def decrease(self, e=None):
        value = int(self.label["text"])
        self.label["text"] = f"{value-int(self.getSpeed())}"
        print("down")