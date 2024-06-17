# ## Use Unity AR to Control and send/receive Pose for Three drones.
# 03/09/2023
# COMPLETED AND WORKING WITH pos (x,z,y) not POSE (x,z,y, rotation)
# NOT TESTED YET...

import math
import time

import UDPCommunication as UDP
import time
from threading import Thread
import numpy as np

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

flight_time = 1   # 2
relative = False
incoming = False

# URI0 = 'radio://0/30/2M/E7E7E7E7E7'  #   (Middle) 30
# URI1 = 'radio://0/40/2M/E7E7E7E7E7'  #   (Left)   40
# URI2 = 'radio://0/39/2M/E7E7E7E7E7'  #   (Right)  39
# URI3 = 'radio://0/10/2M/E7E7E7E7E7'  #   (back)   10
# URI4 = 'radio://0/80/2M/E7E7E7E7E7'  #   (front)  80

# Radio 0 (With Cover)
URI0 = 'radio://0/50/2M/E7E7E7E7E0'  #   (Middle) 30
URI1 = 'radio://0/50/2M/E7E7E7E7E1'  #   (Left)   40
URI2 = 'radio://0/50/2M/E7E7E7E7E2'  #   (Right)  39

# Radio 1 (Naked)
URI3 = 'radio://1/100/2M/E7E7E7E7E3'  #   (back)   10
URI4 = 'radio://1/100/2M/E7E7E7E7E4'  #   (front)  80

uris = [
    URI0,
    URI1,
    URI2,
    URI3,
    URI4,
]

socket = UDP.UdpComms(udpIP="192.168.0.183", portTX=8015, portRX=8016, enableRX=True, suppressWarnings=True)     # Local PC Testing

# == Function =============================================================================================
def receive_data(socket):
    global DroneARCoord1, DroneARCoord2, DroneARCoord3, DroneARCoord4, DroneARCoord5

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
                if len(coordinate_parts) == 15:
                    try:
                        # Convert coordinate parts to floats
                        coordinate_list = [float(part) for part in coordinate_parts]

                        # Separate into DroneARCoord1, DroneARCoord2, and DroneARCoord3
                        DroneARCoord1 = np.array(coordinate_list[:3])
                        DroneARCoord2 = np.array(coordinate_list[3:6])
                        DroneARCoord3 = np.array(coordinate_list[6:9])
                        DroneARCoord4 = np.array(coordinate_list[9:12])
                        DroneARCoord5 = np.array(coordinate_list[12:])
                    
                    except ValueError as e:
                        print(f"ValueError: {e}")
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


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)


def take_off(scf): 
    commander = scf.cf.high_level_commander
    commander.takeoff(0.5, flight_time)
    time.sleep(2)


def landing(scf): 
    commander = scf.cf.high_level_commander
    commander.land(0.0, flight_time)
    time.sleep(5)  
    
def run(scf: SyncCrazyflie, Poses): 
  
    cf = scf.cf
    commander = scf.cf.high_level_commander
    duration = flight_time/2  # 0.1 not tested yet
    x, y, z = Poses[0], Poses[1], Poses[2]

    #print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    time.sleep(0.01)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    
    # if uris.count() == 1:
    #     print("Only One Drone Connected!")
    # elif uris.count() == 2: 
    #     print("Two Drones Connected!!")
    # elif uris.count() == 3: 
    #     print("Three Drones Connected!!!")
    # elif uris.count() == 4: 
    #     print("Four Drones Connected!!!!")

    with Swarm(uris, factory=factory) as swarm:
        try:
            global DroneARCoord1, DroneARCoord2, DroneARCoord3, DroneARCoord4, DroneARCoord5
            DroneARCoord1 = [0.0, 0.0, 0.0]
            DroneARCoord2 = [0.0, 0.0, 0.0]
            DroneARCoord3 = [0.0, 0.0, 0.0]
            DroneARCoord4 = [0.0, 0.0, 0.0]
            DroneARCoord5 = [0.0, 0.0, 0.0]

            print('========= SETTING UP SWARM 5 =============')
            swarm.parallel_safe(activate_high_level_commander)
            print(swarm.get_estimated_positions())
            swarm.reset_estimators()

            # -----------------------------------------------------------------------------------
            exit_flag = False
            udpThread = Thread(target=receive_data, args=(socket,))
            udpThread.start()
            flying = False
            # -----------------------------------------------------------------------------------
            print('==========================================')
            print('All Ready To Go!')
            print('----- Receiving the Poses from Unity -----')

            while True:

                UnityPoseLeader = (DroneARCoord1[0],DroneARCoord1[2],DroneARCoord1[1])
                UnityPoseLeft =   (DroneARCoord2[0],DroneARCoord2[2],DroneARCoord2[1])
                UnityPoseRight =  (DroneARCoord3[0],DroneARCoord3[2],DroneARCoord3[1])
                UnityPoseBack =   (DroneARCoord4[0],DroneARCoord4[2],DroneARCoord4[1])
                UnityPoseFront =  (DroneARCoord5[0],DroneARCoord5[2],DroneARCoord5[1])

                all_poses = {
                    uris[0]: [UnityPoseLeader],
                    uris[1]: [UnityPoseLeft],
                    uris[2]: [UnityPoseRight],
                    uris[3]: [UnityPoseBack],
                    uris[4]: [UnityPoseFront]
                }

                if UnityPoseLeader[2] >= 0.05 or UnityPoseLeft[2] >= 0.05 or UnityPoseRight[2] >= 0.05 or UnityPoseBack[2] >= 0.05 or UnityPoseFront[2] >= 0.05:
                    flying = True
                    swarm.parallel_safe(run, args_dict=all_poses)  
                    # print('Unity Leader Pos: ({}, {}, {})'.format(UnityPoseLeader[0],UnityPoseLeader[1],UnityPoseLeader[2]))
                    # print('Unity Left Pos: ({}, {}, {})'.format(UnityPoseLeft[0],UnityPoseLeft[1],UnityPoseLeft[2]))
                    # print('Unity Right Pos: ({}, {}, {})'.format(UnityPoseRight[0],UnityPoseRight[1],UnityPoseRight[2])) 
                    # print('Unity Back Pos: ({}, {}, {})'.format(UnityPoseBack[0],UnityPoseBack[1],UnityPoseBack[2]))
                    # print('Unity Front Pos: ({}, {}, {})'.format(UnityPoseFront[0],UnityPoseFront[1],UnityPoseFront[2])) 
                        
                
                elif UnityPoseLeader[2] < 0.04 or UnityPoseLeft[2] < 0.04 or UnityPoseRight[2] < 0.04 or UnityPoseFront[2] < 0.04 or UnityPoseBack[2] < 0.04:
                    if (flying == True):
                        swarm.parallel_safe(landing)
                        print('Drone near the ground, landing mode!')
                        flying = False

        except KeyboardInterrupt:
            print("Keyboard Interruptted!  Closing Link.... ")
            swarm.parallel_safe(landing)
            exit_flag = True
            udpThread.join()
            swarm.close_links()
            