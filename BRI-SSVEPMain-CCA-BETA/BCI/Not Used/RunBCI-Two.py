# python RunBCI-Two.py --device menta --trial 24                             <--------- Use this for VIRTUAL DRONE & Crazyflie Only, no feedback, no robot testing.
# python RunBCI-Two.py --device menta --robhost 192.168.0.202 --trial 24     <---- USE THIS FOR FULL SYSTEM (DEMO)

import numpy as np
import time
import os
import sys
import struct

path = os.path.join('.', 'BCI')
sys.path.append(path)

from stream_mindov2 import StreamMindo as mindov2
from stream_mindov1 import StreamMindo as mindov1
from stream_liveamp import StreamLiveAmp as liveamp
from stream_openbci import StreamOpenBCI as openbci
from tcpcomm_two import TCPComm
from stream_v3 import StreamV3 as v3device
from stream_mentalab_two import StreamMenta as menta_two
from stream_mentalab import StreamMenta as menta

import socket
import argparse

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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='port id')
    parser.add_argument('-p', '--port', default='/dev/rfcomm0', help='Port')
    parser.add_argument('-n', '--name', default='s1', help='name')
    parser.add_argument('-s', '--device', default='menta', help='eeg device')
    parser.add_argument('-u', '--user', default=1, help='number of controllers')                      # This is new, add this in TCPcomm_two
    parser.add_argument('-rh', '--robhost', default=None, help='ip of robot')
    parser.add_argument('-t', '--trial', default=24, help='number of trials')
    config = parser.parse_args()

    host = get_ip()
    print("host ip address: ", host)

    fs = 500  # sampling rate of eeg
    T = 5  # duration of eeg buffer# ideally same as ssvep duration
    flickfreq = list(np.arange(9))
    n_trails = int(config.trial)
    print("number of trials =", n_trails)

    device = config.device
    if device == "mindo1":
        fs = 500
        mindo = mindov1("BRA-801", port=config.port, srate=fs, time=T)
    elif device == "mindo2":
        fs = 500
        mindo = mindov2("BRA-801", port=config.port, srate=fs, time=T)
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
        mindo = menta(stream_name="Explore_849F_ExG", srate=fs, channels=8, time=T)
    elif device == "menta-Two":
        fs = 500
        mindo = menta_two(stream_name="Explore_849F_ExG", srate=fs, channels=8, time=T)
    else:
        mindo = None
        print("choose device from 1) mindo1, 2) mindo2, 3)liveamp")
        exit()

    mindo.daemon=True
    mindo.start()
    time.sleep(1)

    if (config.user == 1):
        tcpcomm = TCPComm(mindoobj=mindo, host=host, srate=fs, time=T, savedata=True, session=2, trials=n_trails,
            name=config.name, robohost=config.robhost, usercount=config.user)
        tcpcomm.daemon = True
        tcpcomm.start()
    elif (config.user == 2):
        tcpcomm = TCPComm(mindoobj=mindo, host=host, srate=fs, time=T, savedata=True, session=2, trials=n_trails,
            name=config.name, robohost=config.robhost, usercount=config.user)
        tcpcomm.daemon = True
        tcpcomm.start()

    time.sleep(1)
    Send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
    #########
    # Send_sock.bind(('0.0.0.0', 5000))
    my_ip = socket.gethostbyname(socket.gethostname())
   #my_ip = get_ip()   # This is working on Pi
    Send_sock.bind((my_ip, 5000))
    Send_IP = my_ip
    Leo_IP = "192.168.0.98"
    Phone_IP = "192.168.0.109"   # For Testing on Phone AR
    DellPC_IP = "192.168.0.183"

    Send_PORT_LEO = 5005
    Send_PORT_Drone = 5007
    Send_PORT_Unity = 5105
    Send_PORT_DellPC = 5007

    while True:
        try:
            num = int(input(""))
            MESSAGE = struct.pack('!i', num)
            Send_sock.sendto(MESSAGE, (Send_IP, Send_PORT_Unity))         # <----- SelfIP, PORT: 5005
            Send_sock.sendto(MESSAGE, (Send_IP, Send_PORT_Drone))         # <----- SelfIP, PORT: 5007
            Send_sock.sendto(MESSAGE, (Leo_IP, Send_PORT_LEO))            # <------ For Leo, PORT: 5005   CHANGE THE IP WHEN NECESSARY
            Send_sock.sendto(MESSAGE, (DellPC_IP, Send_PORT_DellPC))
            Send_sock.sendto(MESSAGE, (Phone_IP, 5005)) 
        except KeyboardInterrupt as e:
            print("Got exception:",e)
            break

    sys.exit(0)
    tcpcomm.stoptcp()

