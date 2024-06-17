# Working Script to check the UDP connection between unity and python
# This is meant for Tetsing coordinate of 1 Drone (Leader - Central)
# 23/Jun/2023

import UDPCommunication as U
import time
import numpy as np

# Create UDP socket to use for sending (and receiving)
sock = U.UdpComms(udpIP="192.168.0.202", portTX=8015, portRX=8016, enableRX=True, suppressWarnings=True)

while True:

    data = sock.ReadReceivedData() # read data

    if data != None: # if NEW data has been received since last ReadReceivedData function call
        coordinate_parts = data.strip('()').split(',')
        coordinate_list = [float(part) for part in coordinate_parts]

        DroneARCoord = np.array(coordinate_list)

        correctX = -(DroneARCoord[2])
        correctY =  (DroneARCoord[0])
        correctZ =  (DroneARCoord[1])

        print('Drone AR Pos: ({}, {}, {})'.format(correctX, correctY, correctZ))


    time.sleep(0.1)