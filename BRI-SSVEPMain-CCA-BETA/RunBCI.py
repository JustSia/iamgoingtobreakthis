'''
File: RunBCI.py
Current version done by: Justin Sia
Last Modified: June 22, 2024
Stage: Tested in Simulation, Not yet tested on Holones or Robot
'''
# python RunBCI.py --device mindo2 --port COM7 --trial 24
# python RunBCI.py --device menta --trial 24
# python RunBCI.py --device openbci --trial 24
# python RunBCI.py --device mindo2 --port COM7 --robhost 192.168.168.134 --trial 24

# python RunBCI.py --device menta --trial 24                             <--------- Use this for VIRTUAL/Crazyflie DRONE, no feedback, no robot testing (Cannot be used to send command without robothost).
# python RunBCI.py --device menta --robhost 192.168.0.202 --trial 24     <---- USE THIS FOR FULL SYSTEM (DEMO)
# python RunBCI.py --device menta --robhost 192.168.0.198 --trial 300 --online False  <---- USE THIS FOR EYE SYSTEM (FAKE DEMO, STILL NEED MENTA CONNECTION)
# python RunBCI.py --device menta --robhost 192.168.0.198 --trial 300 --online False --simulate <---- USE THIS FOR EYE SYSTEM (FAKE DEMO, DONT NEED MENTA CONNECTION)
import argparse
import numpy as np
import os
import random
import socket
import struct
import sys
import time

path = os.path.join('.', 'BCI')
sys.path.append(path)

from tcpcomm import TCPComm
from stream_mentalab import StreamMenta as menta
from stream_sim import SimulatedEEG as sim

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

def parse_arguments():
    parser = argparse.ArgumentParser(description='port id')
    parser.add_argument('-p', '--port', default='/dev/rfcomm0', help='Port')
    parser.add_argument('-n', '--name', default='s1', help='name')
    parser.add_argument('-s', '--device', default='menta', help='eeg device')
    parser.add_argument('-rh', '--robhost', default=None, help='ip of robot')
    parser.add_argument('-t', '--trial', default=18, help='number of trials')
    parser.add_argument('-o', '--online', default=True, help='Use EEG Data')
    parser.add_argument('-sim', '--simulate', action='store_true', help='Run in simulation mode')
    return parser.parse_args()

def initialize_device(config):
    device = config.device
    if config.simulate:
        return sim(stream_name="SimulatedEEG", srate=500, channels=8, time=5)
    elif device == "menta":
        return menta(stream_name="Explore_849B_ExG", srate=500, channels=8, time=5)
    else:
        print("Choose device from 1) mindo1, 2) mindo2, 3)liveamp")
        exit()
        
def send_command(config, num):
    MESSAGE = struct.pack('!i', num)
    Send_sock.sendto(MESSAGE, (HoloLens_IP1, Send_PORT_Unity))
    Send_sock.sendto(MESSAGE, (HoloLens_IP2, Send_PORT_Unity))
    Send_sock.sendto(MESSAGE, (HoloLens_IP3, Send_PORT_Unity))
    Send_sock.sendto(MESSAGE, (Send_IP, Send_PORT_Unity))
    Send_sock.sendto(MESSAGE, (Send_IP, Send_PORT_Drone))
    if Send_RoboHost != Send_IP:
        Send_sock.sendto(MESSAGE, (Send_RoboHost, Send_PORT_Drone))
    Send_sock.sendto(MESSAGE, (Leo_IP, Send_PORT_LEO))
    if Send_RoboHost != Leo_IP:
        Send_sock.sendto(MESSAGE, (Send_RoboHost, Send_PORT_LEO))
    Send_sock.sendto(MESSAGE, (DellPC_IP, Send_PORT_DellPC))
    Send_sock.sendto(MESSAGE, (Phone_IP, 5005))

if __name__ == '__main__':
    config = parse_arguments()
    host = get_ip()
    print("Host IP address:", host)

    mindo = initialize_device(config)
    mindo.daemon = True
    mindo.start()
    time.sleep(1)
    print("mindo started")
    tcpcomm = TCPComm(mindoobj=mindo, host=host, srate=500, time=5, savedata=True, session=2,
                      trials=int(config.trial), name=config.name, robohost=config.robhost, online=config.online)
    
    tcpcomm.daemon = True
    tcpcomm.start()
    time.sleep(1)
    
    Send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    my_ip = socket.gethostbyname(socket.gethostname())
    Send_sock.bind((my_ip, 5000))
    Send_IP = my_ip

    Send_RoboHost = config.robhost
    Leo_IP = "192.168.0.98"
    Send_PORT_LEO = 5005
    Send_PORT_Drone = 5007

    HoloLens_IP1 = "192.168.0.41"
    HoloLens_IP2 = "192.168.0.156"
    HoloLens_IP3 = "192.168.0.90"
    Send_PORT_Unity = 5105

    Phone_IP = "192.168.0.109"
    DellPC_IP = "192.168.0.183"
    Send_PORT_DellPC = 5007

    while True:
        try:
            num = int(input(""))
            send_command(config, num)
        except KeyboardInterrupt as e:
            print("Got exception:", e)
            break

    sys.exit(0)
    tcpcomm.stoptcp()

