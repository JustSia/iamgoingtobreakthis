# ## Use Unity AR to Control and send/receive Pose for a Two drones.
# 14/Sep/2023
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
    global DroneARCoord1, DroneARCoord2, DroneARCoord3

    while not exit_flag:
        data = socket.ReadReceivedData()

        if data is not None:
            incoming = True

            # Remove any leading or trailing whitespace
            data = data.strip()

            # Check if the data starts with '(' and ends with ')'
            if data.startswith('(') and data.endswith(')'):
                # Remove the parentheses
                data = data[1:-1]

                # Split the data into coordinate parts
                coordinate_parts = data.split(',')

                # Check if there are exactly 9 coordinate parts
                if len(coordinate_parts) == 9:
                    try:
                        # Convert coordinate parts to floats
                        coordinate_list = [float(part) for part in coordinate_parts]

                        # Separate into DroneARCoord1, DroneARCoord2, and DroneARCoord3
                        DroneARCoord1 = np.array(coordinate_list[:3])
                        DroneARCoord2 = np.array(coordinate_list[3:6])
                        DroneARCoord3 = np.array(coordinate_list[6:])
                        # print('Unity Leader Pos: ({}, {}, {})'.format(DroneARCoord1[0],DroneARCoord1[2],DroneARCoord1[1]))
                        # print('Unity Left Pos: ({}, {}, {})'.format(DroneARCoord2[0],DroneARCoord2[2],DroneARCoord2[1]))
                        # print('Unity Right Pos: ({}, {}, {})'.format(DroneARCoord3[0],DroneARCoord3[2],DroneARCoord3[1])) 
                
                    except ValueError as e:
                        print(f"ValueError: {e}")
                else:
                    print("Data does not contain exactly 9 coordinate parts.")
            else:
                print("Data is not in the expected format (missing parentheses).")



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
        telloswarm = TelloSwarm.fromIps(["192.168.0.6","192.168.0.220"])
        telloswarm.connect(False)
        telloswarm.set_speed(velocity)
        print('Tello Swarm is Ready!! ')
        telloswarm.takeoff()
        #telloswarm.parallel(lambda i, tello: tello.send_command_without_return(go_xyz_speed_mid(UnityX,UnityY,UnityZ,velocity,pad1)))

    
        global DroneARCoord1, DroneARCoord2, DroneARCoord3
        DroneARCoord1 = [0.0, 0.0, 0.0]
        DroneARCoord2 = [0.0, 0.0, 0.0]
        DroneARCoord3 = [0.0, 0.0, 0.0]

        # -----------------------------------------------------------------------------------
        exit_flag = False
        udpThread = Thread(target=receive_data, args=(socket,))
        udpThread.start()
        # -----------------------------------------------------------------------------------

        while True:
            coordinates_array = [None] * 2
            coordinates_array[0] = (int(DroneARCoord1[0] * 100), int(DroneARCoord1[2] * 100), int(DroneARCoord1[1] * 100))
            coordinates_array[1] = (int(DroneARCoord2[0] * 100), int(DroneARCoord2[2] * 100), int(DroneARCoord2[1] * 100))
            print('Drone AR Pos 1: ({}, {}, {})'.format(coordinates_array[0][0], coordinates_array[0][0], coordinates_array[0][0]))
            print('Drone AR Pos 2: ({}, {}, {})'.format(coordinates_array[0][0], coordinates_array[0][0], coordinates_array[0][0]))
            
            if ((DroneARCoord1[1] * 100) >= 10) or ((DroneARCoord2[1] * 100) >= 10):
                telloswarm.parallel(lambda i, tello: tello.go_xyz_speed_mid(coordinates_array[i][0],coordinates_array[i][1],coordinates_array[i][2],velocity,pad1))
                time.sleep(0.1)
                #print('Unity Pos: ({}, {}, {})'.format(UnityX, UnityY, UnityZ)) 
            
            elif ((DroneARCoord1[1] * 100) < 9) or ((DroneARCoord2[1] * 100) < 9):
                telloswarm.land()
                print('Drone near ground, landing mode!')

    except KeyboardInterrupt:
        print("Keyboard Interruptted!  Closing Link.... ")
        exit_flag = True
        udpThread.join()
        #telloswarm.emergency()
