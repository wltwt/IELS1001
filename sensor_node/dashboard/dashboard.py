# https://docs.python.org/3/library/venv.html
# https://pyserial.readthedocs.io/en/latest/shortintro.html
# https://pythonassets.com/posts/tk-after-function-tkinter/
# https://pythonassets.com/posts/background-tasks-with-tk-tkinter/
# https://docs.python.org/3/library/queue.html
# https://tkdocs.com/tutorial/intro.html

# ide hent strømpriser fra nett å se hvor mye penger vi genererer

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
from tkinter import *
from tkinter import ttk
import serial
import queue
import threading
import json

# endre skriftstr graf
plt.rcParams.update({'font.size':4})

# bruker numpy for enkel graf-håndtering
watt_values = np.array([])

# ser etter ny seriell-informasjon kontinuerlig
def serial_worker():
    while True:
        try:
            if ser.in_waiting:          # se om det er noe i seriell-bufferet
                item = ser.readline()   # hent alt som ligger der (frem til \n)
                q.put(item)             # legg det i ett buffer slik at det er tilgjengelig til tkinter
        except ser.SerialException:
            print("Serial connection failed")
            break

# oppdater hoved-vinduet med 100ms intervall
def update_gui():
    
    # kjører løkke og henter ut verdier så lenge det er noe i bufferet
    while not q.empty():
        
        # hent det som ble lest på seriellporten
        item = q.get()
        
        # hent ut verdier 
        json_item = json.loads(item)
        
        # legg verdier i python-variabler
        temp_value = json_item["temp"]
        humidity_value = json_item["humidity"]
        health_value = json_item["health_level"]
        watt_value = json_item["power_produced"]
        time_value = json_item["time_seconds"]
        panel_pressure_value = json_item['pressure']
        
        # hent inn array fra globale skopet
        global watt_values
        
        # sjekk at det er ekte verdier til stede
        if temp_value != 0:
            temp_content.set(f'{round(temp_value, 2)} °C')
            humidity_content.set(f'{round(humidity_value, 2)} %')
            health_content.set(f'{round(health_value, 2)} %')
            watt_values = np.append(watt_values, round(watt_value, 1))
            panel_pressure_content.set(f'{panel_pressure_value} Kg')
            
            # regn ut energi produsert
            mean = np.mean(watt_values)
            kWh = round((mean * time_value) / (3600 * 1000), 3)
            kWh_content.set(f'{kWh} kWh')
            
            # lag ny graf med oppdaterte verdier
            update_graph()

        # for debugging
        print(item)
    root.after(100, update_gui)
    
# fjern forrige graf og sett inn ny info
def update_graph():
    global watt_values
    ax.clear()
    ax.set_title("effekt (W)", fontsize=4, color='white')
    ax.plot(watt_values)
    ax.set_facecolor('#494949')
    ax.tick_params(axis='both', which='major', colors='#FFFFFF')
    canvas.draw()

# reset all innhented informasjon
def reset_values():
    global watt_values
    watt_values = np.array([])
    health_content.set("%")
    temp_content.set("°C")
    humidity_content.set("%")
    kWh_content.set("kWh")
    panel_pressure_content.set("kg")
    update_graph()
    
# avslutt program på enter-tastetrykk
def quit_on_enter(event=None):
    root.destroy()


# sett opp kø for kommunikasjon mellom tkinter og serial-porten
q = queue.Queue()

# opprett seriell-objekt med port, baud-rate og timeout i tilfelle porten bruker for lang tid
ser = serial.Serial('/dev/cu.usbmodem2017_2_251', 9600, timeout=1)

# start thread for kontinuerlig lesing av seriell-porten
threading.Thread(target=serial_worker, daemon=True).start()

# sett opp GUI
root = Tk()
root.geometry('450x650')

# deler opp GUI i rutenett
frm = ttk.Frame(root, padding="25 25 15 25")
frm.grid(column=0, row=0, sticky=(N, W, E, S))

# lag vindutittel
root.title("Solcellepanel oversikt - ver.1.0.0")
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# statiske etiketter
temp_label = ttk.Label(frm, text="Temperatur: ").grid(column=0, row=0, sticky=E)
humidity_label = ttk.Label(frm, text="Relativ luftfuktighet: ").grid(column=0, row=1,sticky=E)
kWh_label = ttk.Label(frm, text="Energi produsert: ").grid(column=0, row=2,sticky=E)
health_label = ttk.Label(frm, text="Helsetilstand: ").grid(column=0, row=3,sticky=E)
panel_pressure_label = ttk.Label(frm, text="Paneltrykk: ").grid(column=0, row=4,sticky=E)


# konfigurer verdier som skal oppdatere seg i vinduet
temp_reading = ttk.Label(frm, text="n/a")
temp_reading.grid(column=1, row=0, sticky=W)
temp_content = StringVar()
temp_reading['textvariable'] = temp_content

humidity_reading = ttk.Label(frm, text="n/a")
humidity_reading.grid(column=1, row=1, sticky=W)
humidity_content = StringVar()
humidity_reading['textvariable'] = humidity_content

kWh_reading = ttk.Label(frm, text="n/a")
kWh_reading.grid(column=1, row=2, sticky=W)
kWh_content = StringVar()
kWh_reading['textvariable'] = kWh_content

health_reading = ttk.Label(frm, text="n/a")
health_reading.grid(column=1, row=3, sticky=W)
health_content = StringVar()
health_reading['textvariable'] = health_content

panel_pressure_reading = ttk.Label(frm, text="n/a")
panel_pressure_reading.grid(column=1, row=4, sticky=W)
panel_pressure_content = StringVar()
panel_pressure_reading['textvariable'] = panel_pressure_content

# lag graf
fig, ax = plt.subplots(figsize=(2.5,2), dpi=100, facecolor='#333333')
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().grid(column=0, row=5, sticky=W)
update_graph()

# lag knapper til GUI
quit_btn = ttk.Button(frm, text="Avslutt", command=root.destroy).grid(column=0, row=6, sticky=E, padx=15, pady=10)
reset_btn = ttk.Button(frm, text="Tilbakestill", command=reset_values).grid(column=2, row=6, sticky=W)

# start oppdaterings-sekvensen til tkinter
root.after(100, update_gui)

# legg til padding rundt alle elementer
for child in frm.winfo_children():
    child.grid_configure(padx=9, pady=5)

# sett opp hotkey for avslutt
root.bind("<Return>", quit_on_enter)

# kjør vindu-program
root.mainloop()