# Working Script to check the UDP connection between unity and python
# This is meant for Tetsing coordinate of 3 Drones
# 03/Aug/2023

import UDPCommunication as UDP
import time
import numpy as np
from threading import Thread

# Create UDP socket to use for sending (and receiving)
socket = UDP.UdpComms(udpIP="192.168.0.202", portTX=8015, portRX=8016, enableRX=True, suppressWarnings=True)     # Local PC Testing

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

                        print('Unity Leader Pos: ({}, {}, {})'.format(DroneARCoord1[0],DroneARCoord1[2],DroneARCoord1[1]))
                        print('Unity Left   Pos: ({}, {}, {})'.format(DroneARCoord2[0],DroneARCoord2[2],DroneARCoord2[1]))
                        print('Unity Right  Pos: ({}, {}, {})'.format(DroneARCoord3[0],DroneARCoord3[2],DroneARCoord3[1])) 

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

            UnityPoseLeader = (DroneARCoord1[0],DroneARCoord1[2],DroneARCoord1[1])
            UnityPoseLeft = (DroneARCoord2[0],DroneARCoord2[2],DroneARCoord2[1])
            UnityPoseRight = (DroneARCoord3[0],DroneARCoord3[2],DroneARCoord3[1])

            if UnityPoseLeader[2] >= 0.05 or UnityPoseLeft[2] >= 0.05 or UnityPoseRight[2] >= 0.05:

                #swarm.parallel_safe(run, args_dict=all_poses)          
                print('Unity Leader Pos: ({}, {}, {})'.format(UnityPoseLeader[0],UnityPoseLeader[1],UnityPoseLeader[2]))
                print('Unity Left Pos: ({}, {}, {})'.format(UnityPoseLeft[0],UnityPoseLeft[1],UnityPoseLeft[2]))
                print('Unity Right Pos: ({}, {}, {})'.format(UnityPoseRight[0],UnityPoseRight[1],UnityPoseRight[2])) 
                

            elif UnityPoseLeader[2] < 0.04 or UnityPoseLeft[2] < 0.04 or UnityPoseRight[2] < 0.04:
                #swarm.parallel_safe(landing)
                #print('Drones close to ground, Landing!!')
                print('Unity Leader Pos: ({}, {}, {})'.format(UnityPoseLeader[0],UnityPoseLeader[1],UnityPoseLeader[2]))
                print('Unity Left   Pos: ({}, {}, {})'.format(UnityPoseLeft[0],UnityPoseLeft[1],UnityPoseLeft[2]))
                print('Unity Right  Pos: ({}, {}, {})'.format(UnityPoseRight[0],UnityPoseRight[1],UnityPoseRight[2])) 

    except KeyboardInterrupt:
        print("Keyboard Interruptted!  Closing Link.... ")
        exit_flag = True
        udpThread.join()
