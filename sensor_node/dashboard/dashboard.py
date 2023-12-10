# https://docs.python.org/3/library/venv.html
# https://pyserial.readthedocs.io/en/latest/shortintro.html
# https://pythonassets.com/posts/tk-after-function-tkinter/
# https://pythonassets.com/posts/background-tasks-with-tk-tkinter/
# https://docs.python.org/3/library/queue.html
# https://tkdocs.com/tutorial/intro.html

from tkinter import *
from tkinter import ttk
import serial
import queue
import threading
import json

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({'font.size':5})
temp_values = np.array([])


# ser etter ny seriell-informasjon kontinuerlig
def serial_worker():
    while True:
        try:
            if ser.in_waiting:
                item = ser.readline()
                q.put(item)
        except ser.SerialException:
            print("Serial connection failed")
            break

# oppdater hoved-vinduet med 100ms intervall
def update_gui(temp_values):
    while not q.empty():
        item = q.get()
        json_item = json.loads(item)
        
        temp_value = json_item["temp"]
        humidity_value = json_item["humidity"]
        

        if temp_value != 0:
            temp_reading.config(text=temp_value)
            
            humidity_reading.config(text=humidity_value)
            temp_values = np.append(temp_values, round(temp_value, 2))
            print(temp_values)
            create_graph(temp_values)

        print(item)
    root.after(100, update_gui, temp_values)
    
    
def create_graph(temp_values):
    fig, ax = plt.subplots(figsize=(2.5,2), dpi=100, facecolor='#333333')
    ax.set_title("effect delivered", fontsize=6, color='white')
    ax.plot(temp_values)
    ax.set_facecolor('#494949')
    ax.tick_params(axis='both', which='major', colors='#FFFFFF')

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.draw()
    canvas.get_tk_widget().grid(column=0, row=2, sticky=(W, S))

    
# avslutt program på enter-tastetrykk
def quit_on_enter(event=None):
    root.destroy()


# sett opp kø for kommunikasjon mellom tkinter og serial-porten
q = queue.Queue()

# opprett serial objekt med port, baud-rate og timeout i tilfelle porten bruker for lang tid
ser = serial.Serial('/dev/cu.usbmodem2017_2_251', 9600, timeout=1)

# start thread for kontinuerlig lesing av serial-porten
threading.Thread(target=serial_worker, daemon=True).start()

# sett opp GUI
root = Tk()
root.geometry('500x500')

# deler opp GUI i rutenett
frm = ttk.Frame(root, padding="3 3 12 12")
frm.grid(column=0, row=0, sticky=(N, W, E, S))

root.title("Sensor dashboard - ver.1.0.0")
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)
root.rowconfigure(1, weight=1)



temp_label = ttk.Label(frm, text="Temperatur: ", width=12).grid(column=0, row=0, sticky=W)
humidity_label = ttk.Label(frm, text="Relativ luftfuktighet: ", width=12).grid(column=0, row=1, sticky=W)


temp_reading = ttk.Label(frm, text="n/a", width=38)
temp_reading.grid(column=1, row=0, sticky=W)

humidity_reading = ttk.Label(frm, text="n/a", width=38)
humidity_reading.grid(column=1, row=1, sticky=W)

create_graph(temp_values)

quit_btn = ttk.Button(frm, text="Avslutt", command=root.destroy).grid(column=1, row=0, sticky=(E))




# create_graph()

root.after(100, update_gui, temp_values)

for child in frm.winfo_children():
    child.grid_configure(padx=9, pady=5)

root.bind("<Return>", quit_on_enter)
root.mainloop()