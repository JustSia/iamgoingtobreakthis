# ## Use Unity AR to Control and send/receive Pose for a SINGLE drone.
# 07/08/2023
# COMPLETED AND WORKING WITH pos (x,z,y) not POSE (x,z,y, rotation)

import math
import time

import UDPCommunication as UDP
import time
from threading import Thread
import numpy as np

from djitellopy import TelloSwarm

flight_time = 2
relative = False
velocity = 50
pad1 = 1

socket = UDP.UdpComms(udpIP="192.168.50.21", portTX=8018, portRX=8019, enableRX=True, suppressWarnings=True)

# == Function =============================================================================================
def receive_data(socket):
    try:
        # ---------- Receive Unity Pose ---------------
        global RPYList
        while not exit_flag:
            data = socket.ReadReceivedData() # read data

            if data != None: # if NEW data has been received since last ReadReceivedData function call
                coordinate_parts = data.strip('()').split(' ')
                RPYL_list = [float(part) for part in coordinate_parts]
                RPYList = np.array(RPYL_list)
                
    except KeyboardInterrupt:
        print("Closing the Socket - Keyboard interrupt")
        socket.close()

    except Exception as e:
        print("Closing Socket - Error:", str(e))

def checkConnection():

    incomingData = socket.ReadReceivedData()              # Send True or False from unity, checking if the unity client is connected to the server (Python)

    if incomingData != 1: 
       status = False 

    if incomingData != None:
        status = True
        print("Connected !!")

    return status


if __name__ == '__main__':

    try:
        print('========== SETTING UP Drone Swarm ==============')
        telloswarm = TelloSwarm.fromIps(["192.168.50.249"])
        telloswarm.connect()
        telloswarm.set_speed(velocity)
        print('Tello Swarm is Ready!! ')
        telloswarm.takeoff()

    
        global RPYList
        RPYList = [0.0, 0.0, 0.0, 0.0]
        #RPYList = [Right, Up, Forward]  # Unity
        #RPYList = [   -Y,  Z,       X]  # Crazyflie

        # -----------------------------------------------------------------------------------
        exit_flag = False
        udpThread = Thread(target=receive_data, args=(socket,))
        udpThread.start()
        # -----------------------------------------------------------------------------------

        while True:
            UnityRoll  = int(RPYList[0] * 100)
            UnityPitch = int(RPYList[1] * 100)    
            UnityYaw   = int(RPYList[2] * 100)
            UnityLift  = int(RPYList[3] * 100) 
            telloswarm.parallel(lambda i, tello: tello.send_rc_control(UnityRoll,UnityPitch,UnityLift,UnityYaw))
            print('Unity RPYL: ({}, {}, {}, {})'.format(UnityRoll, UnityPitch, UnityYaw, UnityLift)) 
            

    except KeyboardInterrupt:
        print("Keyboard Interruptted!  Closing Link.... ")
        exit_flag = True
        udpThread.join()
        telloswarm.emergency()

    except Exception as e:
        exit_flag = True
        print("Error Exception:", str(e))
        udpThread.join()
        telloswarm.emergency()

        
