# ## Use Unity AR to Control and send/receive Pose for a SINGLE drone.
# 07/08/2023
# COMPLETED AND WORKING WITH pos (x,z,y) not POSE (x,z,y, rotation)

import math
import time

import UDPCommunication as UDP
import time
from threading import Thread
import numpy as np

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from simple_pid import PID

flight_time = 2
relative = False
incoming = False

# URI0 = 'radio://0/40/2M/E7E7E7E7E7'  # cf_id 0, startup position [ 0, +0.4 ]     (Left)
# URI1 = 'radio://0/10/2M/E7E7E7E7E7'  # cf_id 1, startup position [ 0,  0.0 ]     (with light)
# URI2 = 'radio://0/80/2M/E7E7E7E7E7'  # cf_id 2, startup position [ 0, -0.4 ]     (Right)
# URI0 = 'radio://0/30/2M/E7E7E7E7E7'

# Radio 0 (With Cover)
URI0 = 'radio://0/50/2M/E7E7E7E7E0'  #   (Middle) 30
URI1 = 'radio://0/50/2M/E7E7E7E7E1'  #   (Left)   40
URI2 = 'radio://0/50/2M/E7E7E7E7E2'  #   (Right)  39

uris = [
    URI0,
    URI1,
    URI2,
]

socket = UDP.UdpComms(udpIP="192.168.0.202", portTX=8015, portRX=8016, enableRX=True, suppressWarnings=True)     # Local PC Testing
#socket = UDP.UdpComms(udpIP="192.168.1.2", portTX=8015, portRX=8016, enableRX=True, suppressWarnings=True)

# == Function =============================================================================================
def receive_data(socket):
    # ---------- Receive Unity Pose ---------------
    global DroneARCoord1, DroneARCoord2, DroneARCoord3, DroneARCoord4, DroneARCoord5
    global dronenum

    while not exit_flag:
        data = socket.ReadReceivedData() # read data

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
                        #print("5")
                        dronenum = 5
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

                if len(coordinate_parts) == 9:
                    try:
                        dronenum = 3
                        #print("3")
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

                if len(coordinate_parts) == 3:
                    try:
                        dronenum = 1
                        print("1")
                        coordinate_list = [float(part) for part in coordinate_parts]
                        # Separate into DroneARCoord1, DroneARCoord2, and DroneARCoord3
                        DroneARCoord1 = np.array(coordinate_list[:3])
                    except ValueError as e:
                        print(f"ValueError: {e}")     


def checkConnection():
    incomingData = socket.ReadReceivedData()              # Send True or False from unity, checking if the unity client is connected to the server (Python)
    if incomingData != 1: 
       status = False 

    if incomingData != None:
        status = True
        print("Connected !!")

    return status


# =============== Drone ==============================
# A callback function for displaying real time position (xyz) data of a specific drone ==================================================================
def position_callback_centre(timestamp, data, logconf):
    global dcx, dcy, dcz
    dcx = data['kalman.stateX']
    dcy = data['kalman.stateY']
    dcz = data['kalman.stateZ']
    print('Drone Leader: ({},{},{})'.format(dcx, dcy, dcz))    

def position_callback_L(timestamp, data, logconf):

    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('Drone Follower (L): ({},{},{})'.format(x, y, z))

def position_callback_R(timestamp, data, logconf):

    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('Drone Follower (R): ({},{},{})'.format(x, y, z)) 

    time.sleep(0.01)            # <--------- Delay the clearing 
    print("\033c", end='')      # <--------- Put this at the end of drone to clear console!!!!!!!!!
    


def start_position_printing(scf):
    print('Start position printing: ')

    if scf.cf.link_uri == 'radio://0/50/2M/E7E7E7E7E0':     # Centre 30                           #  <--------------------- Change URI Here
           
        log_conf = LogConfig(name='Position', period_in_ms=100)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(position_callback_centre)
        log_conf.start()

    ## Add more uris to print different position coordinates for more drosnes
    if scf.cf.link_uri == 'radio://0/50/2M/E7E7E7E7E1':      # Left 40                           #  <--------------------- Change URI Here
         
        log_conf = LogConfig(name='Position', period_in_ms=100)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(position_callback_L)
        log_conf.start()


    if scf.cf.link_uri == 'radio://0/50/2M/E7E7E7E7E2':      # Right 39                          #  <--------------------- Change URI Here
         
        log_conf = LogConfig(name='Position', period_in_ms=100)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(position_callback_R)
        log_conf.start()

    

def activate_stabilisation_estimator(scf):
    scf.cf.param.set_value('stabilizer.estimator', '2')


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf):
    controller = 2  # Use Mellinger
    scf.cf.param.set_value('stabilizer.controller', controller)


def take_off(scf): 
    commander = scf.cf.high_level_commander
    commander.takeoff(0.5, flight_time)
    time.sleep(2)


def landing(scf): 
    commander = scf.cf.high_level_commander
    commander.land(0.0, flight_time)
    time.sleep(10)  
    
def run(scf: SyncCrazyflie, Poses): 
    # Position High   
    cf = scf.cf
    commander = scf.cf.high_level_commander
    duration = flight_time

    # x, y, z = Poses[0][0], Poses[0][1], Poses[0][2]
    # duration = Poses[0][3]

    x, y, z = Poses[0], Poses[1], Poses[2]

    #print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    # print('Highest Middle')
    time.sleep(0.05)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    global dronenum
    # ============== PID ===========================
    # Initialize PID controllers for X, Y, and Z axes
    pid_x = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)  # Adjust Kp, Ki, Kd values accordingly
    pid_y = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)
    pid_z = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=0)

    # Sample time (adjust as needed)
    dt = 0.01  # Assuming 100 Hz
    # ==============================================
    
    with Swarm(uris, factory=factory) as swarm:
        try:
            global DroneARCoord1, DroneARCoord2, DroneARCoord3, DroneARCoord4, DroneARCoord5
            dronenum = 0.0
            DroneARCoord1 = [0.0, 0.0, 0.0]
            DroneARCoord2 = [0.0, 0.0, 0.0]
            DroneARCoord3 = [0.0, 0.0, 0.0]
            DroneARCoord4 = [0.0, 0.0, 0.0]
            DroneARCoord5 = [0.0, 0.0, 0.0]
            
            print('========= SETTING UP DRONE SWARM =============')
            swarm.parallel_safe(activate_high_level_commander)
            swarm.parallel_safe(activate_mellinger_controller)
            swarm.parallel_safe(activate_stabilisation_estimator)
            print(swarm.get_estimated_positions())
            swarm.reset_estimators()

            swarm.parallel_safe(start_position_printing)
            # -----------------------------------------------------------------------------------
            exit_flag = False
            udpThread = Thread(target=receive_data, args=(socket,))
            udpThread.start()
            flying = False
            # -----------------------------------------------------------------------------------
            print('==========================================')
            print('All Ready To Go!')
            #print('Total of '+ str(dronenum) + ' drone(s) connected!')
            print('----- Receiving the Poses from Unity -----')

            while True:
                if dronenum == 5:
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

                elif dronenum == 3:
                    UnityPoseLeader = (DroneARCoord1[0],DroneARCoord1[2],DroneARCoord1[1])
                    UnityPoseLeft =   (DroneARCoord2[0],DroneARCoord2[2],DroneARCoord2[1])
                    UnityPoseRight =  (DroneARCoord3[0],DroneARCoord3[2],DroneARCoord3[1])

                    all_poses = {
                        uris[0]: [UnityPoseLeader],
                        uris[1]: [UnityPoseLeft],
                        uris[2]: [UnityPoseRight]
                    }

                elif dronenum == 1:    
                    global dcx, dcy, dcz
                    pid_output_x = pid_x(dcx - DroneARCoord1[0], dt)
                    pid_output_y = pid_y(dcy - DroneARCoord1[2], dt)
                    pid_output_z = pid_z(dcz - DroneARCoord1[1], dt)
                    UnityPoseLeader = (pid_output_x,pid_output_y,pid_output_z)
                    
                    all_poses = {
                        uris[0]: [UnityPoseLeader],
                    }

                if UnityPoseLeader[2] >= 0.05:
                    flying = True
                    swarm.parallel_safe(run, args_dict=all_poses)  
                    # print('Unity Leader Pos: ({}, {}, {})'.format(UnityPoseLeader[0],UnityPoseLeader[1],UnityPoseLeader[2]))
                    # print('Unity Left Pos: ({}, {}, {})'.format(UnityPoseLeft[0],UnityPoseLeft[1],UnityPoseLeft[2]))
                    # print('Unity Right Pos: ({}, {}, {})'.format(UnityPoseRight[0],UnityPoseRight[1],UnityPoseRight[2])) 
                    # print('Unity Back Pos: ({}, {}, {})'.format(UnityPoseBack[0],UnityPoseBack[1],UnityPoseBack[2]))
                    # print('Unity Front Pos: ({}, {}, {})'.format(UnityPoseFront[0],UnityPoseFront[1],UnityPoseFront[2])) 
                        
                
                elif UnityPoseLeader[2] < 0.04:
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
            
