# ## Use Unity AR to Control and send/receive Pose for a SINGLE drone.
# 07/08/2023
# COMPLETED AND WORKING WITH pos (x,z,y) not POSE (x,z,y, rotation)

# https://djitellopy.readthedocs.io/en/latest/swarm/
# https://dl.djicdn.com/downloads/RoboMaster+TT/Tello_SDK_3.0_User_Guide_en.pdf

import math
import time

import UDPCommunication as UDP
import time
from threading import Thread
import numpy as np

from djitellopy import TelloSwarm

flight_time = 2
relative = False
velocity = 100
pad1 = 1

socket = UDP.UdpComms(udpIP="192.168.0.202", portTX=8015, portRX=8016, enableRX=True, suppressWarnings=True)

# == Function =============================================================================================
def receive_data(socket):
    try:
        # ---------- Receive Unity Pose ---------------
        global DroneARCoord
        while not exit_flag:
            data = socket.ReadReceivedData() # read data

            if data != None: # if NEW data has been received since last ReadReceivedData function call
                coordinate_parts = data.strip('()').split(',')
                coordinate_list = [float(part) for part in coordinate_parts]
                DroneARCoord = np.array(coordinate_list)
                #print('Pose (Inside): ({}, {}, {})'.format(-(DroneARCoord[2]), DroneARCoord[0], DroneARCoord[1]))

    except KeyboardInterrupt:
        print("closing the socket")
        socket.close()

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
        print('========== SETTING UP Tello Swarm ==============')
        telloswarm = TelloSwarm.fromIps(["192.168.0.220"])
        telloswarm.connect(False)
        telloswarm.set_speed(velocity)
        print('Tello Swarm is Ready!! ')
        telloswarm.takeoff()
        #telloswarm.parallel(lambda i, tello: tello.send_command_without_return(go_xyz_speed_mid(UnityX,UnityY,UnityZ,velocity,pad1)))

    
        global DroneARCoord
        DroneARCoord = [0.0, 0.0, 0.0]
        #DroneARCoord = [Right, Up, Forward]  # Unity
        #DroneARCoord = [   -Y,  Z,       X]  # Crazyflie

        # -----------------------------------------------------------------------------------
        exit_flag = False
        udpThread = Thread(target=receive_data, args=(socket,))
        udpThread.start()
        # -----------------------------------------------------------------------------------

        while True:
            # UnityX =  int(DroneARCoord[2] * 100)  
            # UnityY = -int(DroneARCoord[0] * 100)    
            # UnityZ =  int(DroneARCoord[1] * 100) 
            UnityX =  int(DroneARCoord[0] * 100)  
            UnityY =  int(DroneARCoord[2] * 100)    
            UnityZ =  int(DroneARCoord[1] * 100) 

            UnityPose = (UnityX,UnityY,UnityZ)
            # print('Drone AR Pos: ({}, {}, {})'.format(correctX, correctY, correctZ))

            if UnityZ >= (10):
                telloswarm.go_xyz_speed_mid(UnityX,UnityY,UnityZ,velocity,pad1)      
                print('Unity Pos: ({}, {}, {})'.format(UnityX, UnityY, UnityZ)) 
            
            elif UnityZ < (9):
                telloswarm.land()
                print('Drone near ground, landing mode!')

    except KeyboardInterrupt:
        print("Keyboard Interruptted!  Closing Link.... ")
        exit_flag = True
        udpThread.join()
        #telloswarm.emergency()
