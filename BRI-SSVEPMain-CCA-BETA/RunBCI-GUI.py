import numpy as np
import time
import sys
import struct
import os

path = os.path.join('.', 'BCI')
sys.path.append(path)

from stream_mindov2 import StreamMindo as mindov2
from stream_mindov1 import StreamMindo as mindov1
from stream_liveamp import StreamLiveAmp as liveamp
from stream_openbci import StreamOpenBCI as openbci
from udpcomm import TCPComm
from stream_v3 import StreamV3 as v3device
from stream_mentalab import StreamMenta as menta

import socket
import argparse
import tkinter as tk
from tkinter import ttk
import re

def on_closing():
    sys.exit(0)

def manu_command(entry,Send_sock):
    global robhost_list
    function_value = entry.get()
    for Send_IP in robhost_list:
        num = int(function_value)
        MESSAGE = struct.pack('!i', num)
        Send_sock.sendto(MESSAGE, (Send_IP, 5005))
    # Add your custom functionality here
    print(f"Manually sending to: {function_value}")

def is_valid_ip(ip_str):
    # Define a regular expression pattern for matching IPv4 addresses
    ip_pattern = r"^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$"

    # Use the re.match function to check if the string matches the pattern
    if re.match(ip_pattern, ip_str):
        return True
    else:
        return False
    
def add_robhost(robhost_listbox):
    global robhost_list,robhost_entry,robport_list,port_entry
    # Get the Robhost value from the entry and add it to the list
    robhost_value = robhost_entry.get()
    robport_value = port_entry.get()
    
    if robhost_value and is_valid_ip(robhost_value) and not (robhost_value in robhost_list and robport_value in robport_list):
        robhost_list.append(robhost_value)
        robport_list.append(robport_value)
    update_robhost_listbox(robhost_list,robport_list,robhost_listbox,)


def pop_robhost(robhost_listbox):
    global robhost_list,robhost_entry,robport_list,port_entry
    robhost_list.pop()
    robport_list.pop()
    update_robhost_listbox(robhost_list,robport_list,robhost_listbox,)

def empty_robhost(robhost_listbox):
    global robhost_list,robhost_entry,robport_list,port_entry
    robhost_list = []
    robport_list = []
    update_robhost_listbox(robhost_list,robport_list,robhost_listbox,)


def update_robhost_listbox(list,port_list,robhost_listbox):
    # Clear the existing listbox and update it with the current list of Robhosts
    robhost_listbox.delete(0, tk.END)
    
    for i, robhost in enumerate(list, start=1):
        robhost_listbox.insert(tk.END, robhost+":"+port_list[i-1])
        # robhost_listbox.insert(robhost)


def ModeChange(*args):
    global tcpcomm,robhost_list,robport_list
    mode = mode_var.get()
    
    print("======================================")

    if mode == "Virtual Drones":
        print("VD Mode")
        robhost_list = []
        robport_list = []
        update_robhost_listbox(robhost_list,robport_list,robhost_listbox,)

    elif mode == "Tello":
        print("Tello Mode")
        robhost_list = []
        robport_list = []
        robhost_list.append("192.168.0.183")
        robport_list.append("5007")
        robhost_list.append("192.168.0.202")
        robport_list.append("5007")
        update_robhost_listbox(robhost_list,robport_list,robhost_listbox,)

    elif mode == "LeoRover":
        print("Leo Mode")
        robhost_list = []
        robport_list = []
        robhost_list.append("192.168.0.98")
        robport_list.append("5005")
        update_robhost_listbox(robhost_list,robport_list,robhost_listbox,)

    elif mode == "Collab Mode":
        print("Collab Mode")
        robhost_list = []
        robport_list = []
        robhost_list.append("192.168.0.98")
        robport_list.append("5005")
        robhost_list.append("192.168.0.202")
        robport_list.append("5007")
        robhost_list.append("192.168.0.183")
        robport_list.append("5007")  
        update_robhost_listbox(robhost_list,robport_list,robhost_listbox,)   

    elif mode == "Phone": 
        robhost_list = []
        robport_list = []
        robhost_list.append("192.168.0.109")
        robport_list.append("5105")
        update_robhost_listbox(robhost_list,robport_list,robhost_listbox,)   

    
def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


def run_script(port_entry,name_entry,device_var,robhost_entry,trial_entry,menta_var):
    global tcpcomm,robhost_list,robport_list
    
    if not tcpcomm:

        fs = 500  # sampling rate of eeg
        T = 5  # duration of eeg buffer# ideally same as ssvep duration
        flickfreq = list(np.arange(9))

        port = port_entry.get()
        name = name_entry.get()
        device = device_var.get()
        # robhost = robhost_entry.get()

        if len(robhost_list) == 0:
            robhost = None
            print("======================================")
            print("No Robot Host Mode")
        else:
            robhost = robhost_list
            for item in robhost_list:    
                print("Robot Host List: ", item)

        print("======================================")

        host = get_ip()
        print("host ip address: ", host)

        n_trails = trial_entry.get()
        stream_name = menta_var.get()
        print("Number of Trials = ", n_trails)

        device = device
    
        if device == "mindo1":
            fs = 500
            mindo = mindov1("BRA-801", port=port, srate=fs, time=T)
        elif device == "mindo2":
            fs = 500
            mindo = mindov2("BRA-801", port=port, srate=fs, time=T)
        elif device == "liveamp":
            fs = 500
            mindo = liveamp(stream_name="LiveAmpSN-055606-0346", srate=fs, channels=64, time=T)
        elif device == "openbci":
            fs = 250
            mindo = openbci(stream_name="obci_eeg1", srate=fs, channels=8, time=T)
        elif device == "v3":
            fs = 250
            mindo = v3device(stream_name="Cyton8_BFSample", srate=fs, channels=4, time=T)
        elif device == "menta":
            fs = 500
            mindo = menta(stream_name=stream_name, srate=fs, channels=8, time=T)
        else:
            mindo = None
            print("choose device from 1) mindo1, 2) mindo2, 3)liveamp")
            exit()
    
        mindo.daemon = True
        mindo.start()
        time.sleep(1)
        tcpcomm = TCPComm(mindoobj=mindo, host=host, srate=fs, time=T, savedata=True, session=2, trials=n_trails,
                          name=name, robohost=robhost,robports=robport_list)

        # tcpcomm = UDPComm(mindoobj=mindo, host=host, srate=fs, time=T, savedata=True, session=2, trials=24,
        #                   name=config.name, robohost=config.robhost)
        tcpcomm.daemon = True
        tcpcomm.start()

        # mindo.join()
        # tcpcomm.join()
        time.sleep(1)

        # Send_PORT = 5005
        # while True:
        #     try:
        #         num = int(input(""))
        #         MESSAGE = struct.pack('!i', num)
        #         Send_sock.sendto(MESSAGE, (Send_IP, Send_PORT))
        #     except KeyboardInterrupt as e:
        #         print("Got exception:",e)
        #         break
        # sys.exit(0)
        # tcpcomm.stoptcp()
        run_button.config(text="Stop Script")

    else:
        tcpcomm.stop()
        tcpcomm.stoptcp()
        run_button.config(text="Start Script")
        tcpcomm = None
        # robhost_list=[]
        # robport_list=[]


###python main.py --device mindo2 --port COM7
if __name__ == '__main__':
    global tcpcomm,robhost_list,robhost_entry,robport_list,port_entry
    proc = None
    robhost_list=[]
    robport_list=[]
    tcpcomm = None
    # Create the main window
    root = tk.Tk()
    root.title("BRI Script GUI")

    # Create labels and input fields for each argument
    port_label = ttk.Label(root, text="Port:")
    port_entry = ttk.Entry(root, width=30)
    port_entry.insert(0, "/dev/rfcomm0")  # Default value

    name_label = ttk.Label(root, text="Name:")
    name_entry = ttk.Entry(root, width=30)
    name_entry.insert(0, "Liang")  # Default value

    # robhost_label = ttk.Label(root, text="Robhost (comma-separated):")
    # robhost_entry = ttk.Entry(root, width=30)
    # robhost_entry.insert(0,  get_ip())  # Default values, separated by commas

    # Create a label, entry, and button for adding Robhosts
    robhost_label = ttk.Label(root, text="Add Robhost:")

    robhost_label.grid(row=0, column=0, padx=10, pady=5, sticky="E")
    robhost_entry = ttk.Entry(root, width=30)
    #robhost_entry.insert(0, socket.gethostbyname(socket.gethostname()))
    robhost_entry.insert(0, get_ip())    # PI
    robhost_entry.grid(row=0, column=1, padx=10, pady=5,)
    robhost_listbox = tk.Listbox(root, width=40, height=10)
    robhost_listbox.grid(row=2, column=0, columnspan=3, padx=10, pady=5)

    port_label = ttk.Label(root, text="Port:")
    port_label.grid(row=0, column=2, padx=10, pady=5, sticky="E")

    port_entry = ttk.Entry(root, width=10)
    port_entry.insert(0,5005)
    port_entry.grid(row=0, column=3, padx=10, pady=5)


    add_button = ttk.Button(root, text="Add", command=lambda: add_robhost(robhost_listbox))
    add_button.grid(row=2, column=3, padx=10, pady=5)

    add_button = ttk.Button(root, text="Delete", command=lambda: pop_robhost(robhost_listbox))
    add_button.grid(row=3, column=3, padx=10, pady=5)

    add_button = ttk.Button(root, text="Empty", command=lambda: empty_robhost(robhost_listbox))
    add_button.grid(row=4, column=3, padx=10, pady=5)

    # Create a listbox to display the added Robhosts

    # Create a listbox to display the added Robhosts
    # robhost_listbox = tk.Listbox(root, width=40, height=10)

    # Arrange the widgets in the window

    trial_label = ttk.Label(root, text="Trial:")
    trial_entry = ttk.Entry(root, width=30)
    trial_entry.insert(0, "24")  # Default value

    device_label = ttk.Label(root, text="Device:")
    device_var = tk.StringVar(root)
    device_var.set("menta")
    device_choices = ["menta", "mindo2", "openbci"]
    device_combobox = ttk.OptionMenu(root, device_var, device_var.get(), *device_choices)
    
    # Default value
    # Create a Combobox for stream_name options for "menta" device
    menta_label = ttk.Label(root, text="Menta Stream:")
    menta_var = tk.StringVar(root)
    menta_var.set("Explore_849F_ExG")
    menta_choices = ["Explore_849F_ExG", "Explore_849B_ExG"]
    menta_combobox = ttk.OptionMenu(root, menta_var, menta_var.get(), *menta_choices)

    # ----------- Add Mode for Virtual Drones Toggle --------------------
    
    mode_label = ttk.Label(root, text="Mode:")
    mode_var = tk.StringVar(root)
    mode_var.set("Virtual Drones")
    mode_choices = ["Virtual Drones", "Tello", "LeoRover", "Collab Mode", "Phone"]
    mode_combobox = ttk.OptionMenu(root, mode_var, mode_var.get(), *mode_choices)
    mode_var.trace_add("write", ModeChange)

    # -------------------------------------------------------------------

    run_button = ttk.Button(root, text="Start Script", command=lambda: run_script(port_entry,name_entry,device_var,robhost_entry,trial_entry,menta_var))
    # robhost_label.grid(row=2, column=0, padx=10, pady=5, sticky="E")
    # robhost_entry.grid(row=2, column=1, padx=10, pady=5)

    # Create labels, entry, and button for adding Robhosts using grid



    trial_label.grid(row=3, column=0, padx=10, pady=5, sticky="E")
    trial_entry.grid(row=3, column=1, padx=10, pady=5)

    device_label.grid(row=4, column=0, padx=10, pady=5, sticky="E")
    device_combobox.grid(row=4, column=1, padx=10, pady=5)

    menta_label.grid(row=5, column=0, padx=10, pady=5, sticky="E")
    menta_combobox.grid(row=5, column=1, padx=10, pady=5)

    mode_label.grid(row=1, column=0, padx=10, pady=5, sticky="E")
    mode_combobox.grid(row=1, column=1, padx=10, pady=5)
    
    run_button.grid(row=7, column=0, pady=10, sticky="E")

    # Create labels, entry, and button for the new functionality
    new_function_label = ttk.Label(root, text="Manual Target:")
    new_function_label.grid(row=7, column=1, padx=10, pady=5, sticky="E")

    new_function_entry = ttk.Entry(root, width=5)
    new_function_entry.grid(row=7, column=2, padx=10, pady=5)
    new_function_entry.insert(0, 1)
    Send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    #my_ip = socket.gethostbyname(socket.gethostname())
    my_ip = get_ip()   # This is working on Pi 
    Send_sock.bind((my_ip, 5000))

    new_function_button = ttk.Button(root, text="Send",
                                     command=lambda: manu_command(new_function_entry,Send_sock))
    new_function_button.grid(row=7, column=3, padx=10, pady=5)
    root.protocol("WM_DELETE_WINDOW", on_closing)

    # Start the GUI main loop
    root.mainloop()


