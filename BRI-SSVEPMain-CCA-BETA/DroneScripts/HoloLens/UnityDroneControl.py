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
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

flight_time = 2
relative = False
incoming = False

# URI0 = 'radio://0/40/2M/E7E7E7E7E7'  # cf_id 0, startup position [ 0, +0.4 ]     (Left)
# URI1 = 'radio://0/10/2M/E7E7E7E7E7'  # cf_id 1, startup position [ 0,  0.0 ]     (with light)
# URI2 = 'radio://0/80/2M/E7E7E7E7E7'  # cf_id 2, startup position [ 0, -0.4 ]     (Right)
# URI0 = 'radio://0/30/2M/E7E7E7E7E7'

URI0 = 'radio://0/50/2M/E7E7E7E7E2'  #   (Middle) 30

uris = [
    URI0,
    # URI1,
    # URI2,
]

socket = UDP.UdpComms(udpIP="192.168.0.202", portTX=8015, portRX=8016, enableRX=True, suppressWarnings=True)     # Local PC Testing
#socket = UDP.UdpComms(udpIP="192.168.1.2", portTX=8015, portRX=8016, enableRX=True, suppressWarnings=True)

# == Function =============================================================================================
def receive_data(socket):
    # ---------- Receive Unity Pose ---------------
    global DroneARCoord
    # DroneARCoord = [0.0, 0.0, 0.0]
    while not exit_flag:
        data = socket.ReadReceivedData() # read data

        if data != None: # if NEW data has been received since last ReadReceivedData function call
            incoming = True
            coordinate_parts = data.strip('()').split(',')
            coordinate_list = [float(part) for part in coordinate_parts]
            DroneARCoord = np.array(coordinate_list)
            print('Unity Leader Pos: ({}, {}, {})'.format(DroneARCoord[0],DroneARCoord[1],DroneARCoord[2]))
        else: 
            print("nah")

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
            global DroneARCoord
            DroneARCoord = [0.0, 0.0, 0.0]
            
            print('========= SETTING UP DRONE 1 =============')
            swarm.parallel_safe(activate_high_level_commander)
            print(swarm.get_estimated_positions())
            swarm.reset_estimators()

            # -----------------------------------------------------------------------------------
            exit_flag = False
            udpThread = Thread(target=receive_data, args=(socket,))
            udpThread.start()
            # -----------------------------------------------------------------------------------
            print('==========================================')
            print('All Ready To Go!')
            print('----- Receiving the Poses from Unity -----')

            while True:
                # UnityX =  (DroneARCoord[2])   
                # UnityY = -(DroneARCoord[0])   
                # UnityZ =  (DroneARCoord[1])
                UnityX =  (DroneARCoord[0])   
                UnityY =  (DroneARCoord[2])   
                UnityZ =  (DroneARCoord[1])
                UnityPose = (UnityX,UnityY,UnityZ)

                all_poses = {
                    uris[0]: [UnityPose]
                }

                if UnityZ >= 0.05:

                    swarm.parallel_safe(run, args_dict=all_poses)          
                    print('Unity Pos: ({}, {}, {})'.format(UnityX, UnityY, UnityZ)) 
                
                elif UnityZ < 0.04:
                    swarm.parallel_safe(landing)
                    print('Drone near ground, landing mode!')

        except KeyboardInterrupt:
            print("Keyboard Interruptted!  Closing Link.... ")
            exit_flag = True
            udpThread.join()
            swarm.parallel_safe(landing)
            swarm.close_links()
            




# # ## Use Unity AR to Control and send/receive Pose for a SINGLE drone.
# # 18/07/2023
# # NOT COMPLETE YET!!!!!!

# # Optimise: 
# #   1. 
# #   2. 
# #   3. 

# import math
# import time

# import UDPCommunication as UDP
# import time
# from threading import Thread
# import numpy as np

# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.log import LogConfig
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.syncLogger import SyncLogger
# from cflib.utils import uri_helper

# uri = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E7')

# socket = UDP.UdpComms(udpIP="127.0.0.1", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)


# def wait_for_position_estimator(scf):
#     print('Waiting for estimator to find position...')

#     log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
#     log_config.add_variable('kalman.varPX', 'float')
#     log_config.add_variable('kalman.varPY', 'float')
#     log_config.add_variable('kalman.varPZ', 'float')

#     var_y_history = [1000] * 10
#     var_x_history = [1000] * 10
#     var_z_history = [1000] * 10

#     threshold = 0.001

#     with SyncLogger(scf, log_config) as logger:
#         for log_entry in logger:
#             data = log_entry[1]

#             var_x_history.append(data['kalman.varPX'])
#             var_x_history.pop(0)
#             var_y_history.append(data['kalman.varPY'])
#             var_y_history.pop(0)
#             var_z_history.append(data['kalman.varPZ'])
#             var_z_history.pop(0)

#             min_x = min(var_x_history)
#             max_x = max(var_x_history)
#             min_y = min(var_y_history)
#             max_y = max(var_y_history)
#             min_z = min(var_z_history)
#             max_z = max(var_z_history)

#             print("{} {} {}".
#                   format(max_x - min_x, max_y - min_y, max_z - min_z))

#             if (max_x - min_x) < threshold and (
#                     max_y - min_y) < threshold and (
#                     max_z - min_z) < threshold:
#                 break


# def set_initial_position(scf, x, y, z, yaw_deg):
#     scf.cf.param.set_value('kalman.initialX', x)
#     scf.cf.param.set_value('kalman.initialY', y)
#     scf.cf.param.set_value('kalman.initialZ', z)

#     yaw_radians = math.radians(yaw_deg)
#     scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


# def reset_estimator(scf):
#     cf = scf.cf
#     cf.param.set_value('kalman.resetEstimation', '1')
#     time.sleep(0.1)
#     cf.param.set_value('kalman.resetEstimation', '0')

#     wait_for_position_estimator(cf)



# def position_callback(timestamp, data, logconf):
#     x = data['kalman.stateX']
#     y = data['kalman.stateY']
#     z = data['kalman.stateZ']
#     # print('Drone Pos: ({}, {}, {})'.format(x, y, z))

#     # ---------- Sending Pose to Unity -------------

#     # DronePos = [round(x,2), round(y,2), round(z,2)]
#     # toSend = ','.join(map(str, DronePos)) # Converting Vector3 to a string, example "0,0,0"
#     # # print("To Send: " + toSend) 
#     # socket.SendData(toSend) 

#     # ---------- Receive Unity Pose ---------------
#     global DroneARCoord
#     DroneARCoord = [0.0, 0.0, 0.0]
    
#     data = socket.ReadReceivedData() # read data

#     if data != None: # if NEW data has been received since last ReadReceivedData function call
#         coordinate_parts = data.strip('()').split(',')
#         coordinate_list = [float(part) for part in coordinate_parts]
#         DroneARCoord = np.array(coordinate_list)
#         print('Drone AR Pos: ({}, {}, {})'.format(-(DroneARCoord[2]), DroneARCoord[0], DroneARCoord[1]))
#     time.sleep(0.1)


# def start_position_printing(scf):
#     log_conf = LogConfig(name='Position', period_in_ms=100)
#     log_conf.add_variable('kalman.stateX', 'float')
#     log_conf.add_variable('kalman.stateY', 'float')
#     log_conf.add_variable('kalman.stateZ', 'float')

#     scf.cf.log.add_config(log_conf)
#     log_conf.data_received_cb.add_callback(position_callback)
#     log_conf.start()


# # == Function =============================================================================================
# def checkConnection():

#     incomingData = socket.ReadReceivedData()              # Send True or False from unity, checking if the unity client is connected to the server (Python)

#     if incomingData != 1: 
#        status = False 

#     if incomingData != None:
#         status = True
#         print("Connected !!")

#     return status



# if __name__ == '__main__':

#     cflib.crtp.init_drivers()

#     with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

#         # set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
#         # reset_estimator(scf)            # Only with Optical Sensor
#         start_position_printing(scf)

#         cf = scf.cf
#         flight_time = 2
#         range = 0.4
#         global DroneARCoord
#         DroneARCoord = [0.0, 0.0, 0.0]

#         # -----------------------------------------------------------------------------------
#         cf.param.set_value('locSrv.extQuatStdDev', 8.0e-3)
#         cf.param.set_value('stabilizer.estimator', '2')
#         cf.param.set_value('commander.enHighLevel', '1')     # Activate high level commander
#         cf.param.set_value('stabilizer.controller', '2')

#         # -----------------------------------------------------------------------------------

#         while True:
#             # data = socket.ReadReceivedData() # read data

#             # if data != None: # if NEW data has been received since last ReadReceivedData function call
#             #     coordinate_parts = data.strip('()').split(',')
#             #     coordinate_list = [float(part) for part in coordinate_parts]

#             #     DroneARCoord = np.array(coordinate_list)

#             correctX = -(DroneARCoord[2])
#             correctY =  (DroneARCoord[0])
#             correctZ =  (DroneARCoord[1])

#             #     print('Drone AR Pos: ({}, {}, {})'.format(correctX, correctY, correctZ))
#             # else:
#             #     print("Not Connected to the Unity Client...  Reconnecting .... ")
            
#             # time.sleep(0.1)

#             usrInput = input('User Command: ')

#             if usrInput == 't':
#                 # print('t')
#                 cf.high_level_commander.takeoff(0.3, flight_time)
#                 time.sleep(2)

#             elif usrInput == 'l':
#                 cf.high_level_commander.land(0.0, flight_time)
#                 time.sleep(2) 

#             elif usrInput == 'y':
#                 cf.high_level_commander.go_to(correctX, correctY, correctZ, 0, flight_time, relative=False)
#                 time.sleep(0.1)

#             if usrInput == '.':
#                 print('end of program.. ')
#                 cf.high_level_commander.land(0.0, flight_time)
#                 break






# # ## Use Unity AR to Control and send/receive Pose for a SINGLE drone.
# # 18/07/2023
# # NOT COMPLETE YET!!!!!!

# # Optimise: 
# #   1. 
# #   2. 
# #   3. 

# import math
# import time

# import UDPCommunication as UDP
# import time
# from threading import Thread
# import numpy as np

# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.log import LogConfig
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.syncLogger import SyncLogger
# from cflib.utils import uri_helper

# uri = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E7')

# socket = UDP.UdpComms(udpIP="127.0.0.1", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)

# class UDPUnity(Thread):

#     def __init__(self,drone):
#         Thread.__init__(self)

#         self.position = [0.0,0.0,0.0]
#         self.x = 0.0
#         self.y = 0.0
#         self.z = 0.0

#         self.start()
#         print('UDP Thread Started')

#     def close(self):
#         self.join()
#         print('UDP Thread Closed')


#     def ReadUDPPose(self):

#         data = socket.ReadReceivedData() # read data

#         if data != None: # if NEW data has been received since last ReadReceivedData function call
#             coordinate_parts = data.strip('()').split(',')
#             coordinate_list = [float(part) for part in coordinate_parts]

#             DroneARCoord = np.array(coordinate_list)

#             correctX = -(DroneARCoord[2])
#             correctY =  (DroneARCoord[0])
#             correctZ =  (DroneARCoord[1])

#             print('Drone AR Pos: ({}, {}, {})'.format(correctX, correctY, correctZ))

#         time.sleep(0.1)  

#         return DroneARCoord      
    

# def wait_for_position_estimator(scf):
#     print('Waiting for estimator to find position...')

#     log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
#     log_config.add_variable('kalman.varPX', 'float')
#     log_config.add_variable('kalman.varPY', 'float')
#     log_config.add_variable('kalman.varPZ', 'float')

#     var_y_history = [1000] * 10
#     var_x_history = [1000] * 10
#     var_z_history = [1000] * 10

#     threshold = 0.001

#     with SyncLogger(scf, log_config) as logger:
#         for log_entry in logger:
#             data = log_entry[1]

#             var_x_history.append(data['kalman.varPX'])
#             var_x_history.pop(0)
#             var_y_history.append(data['kalman.varPY'])
#             var_y_history.pop(0)
#             var_z_history.append(data['kalman.varPZ'])
#             var_z_history.pop(0)

#             min_x = min(var_x_history)
#             max_x = max(var_x_history)
#             min_y = min(var_y_history)
#             max_y = max(var_y_history)
#             min_z = min(var_z_history)
#             max_z = max(var_z_history)

#             print("{} {} {}".
#                   format(max_x - min_x, max_y - min_y, max_z - min_z))

#             if (max_x - min_x) < threshold and (
#                     max_y - min_y) < threshold and (
#                     max_z - min_z) < threshold:
#                 break


# def set_initial_position(scf, x, y, z, yaw_deg):
#     scf.cf.param.set_value('kalman.initialX', x)
#     scf.cf.param.set_value('kalman.initialY', y)
#     scf.cf.param.set_value('kalman.initialZ', z)

#     yaw_radians = math.radians(yaw_deg)
#     scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


# def reset_estimator(scf):
#     cf = scf.cf
#     cf.param.set_value('kalman.resetEstimation', '1')
#     time.sleep(0.1)
#     cf.param.set_value('kalman.resetEstimation', '0')

#     wait_for_position_estimator(cf)



# def position_callback(timestamp, data, logconf):
#     x = data['kalman.stateX']
#     y = data['kalman.stateY']
#     z = data['kalman.stateZ']
#     # print('Drone Pos: ({}, {}, {})'.format(x, y, z))
#     DronePos = [round(x,2), round(y,2), round(z,2)]
#     toSend = ','.join(map(str, DronePos)) # Converting Vector3 to a string, example "0,0,0"
#     # print("To Send: " + toSend) 
#     socket.SendData(toSend) 


# def start_position_printing(scf):
#     log_conf = LogConfig(name='Position', period_in_ms=500)
#     log_conf.add_variable('kalman.stateX', 'float')
#     log_conf.add_variable('kalman.stateY', 'float')
#     log_conf.add_variable('kalman.stateZ', 'float')

#     scf.cf.log.add_config(log_conf)
#     log_conf.data_received_cb.add_callback(position_callback)
#     log_conf.start()


# # == Function =============================================================================================
# def checkConnection():

#     incomingData = socket.ReadReceivedData()              # Send True or False from unity, checking if the unity client is connected to the server (Python)

#     if incomingData != 1: 
#        status = False 

#     if incomingData != None:
#         status = True
#         print("Connected !!")

#     return status



# if __name__ == '__main__':

#     cflib.crtp.init_drivers()

#     with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

#         # set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
#         reset_estimator(scf)            # Only with Optical Sensor
#         start_position_printing(scf)

#         cf = scf.cf
#         flight_time = 2
#         range = 0.4

#         # -----------------------------------------------------------------------------------
#         cf.param.set_value('locSrv.extQuatStdDev', 8.0e-3)
#         cf.param.set_value('stabilizer.estimator', '2')
#         cf.param.set_value('commander.enHighLevel', '1')     # Activate high level commander
#         cf.param.set_value('stabilizer.controller', '2')

#         UDP_warpper = UDPUnity(cf)
#         # -----------------------------------------------------------------------------------

#         while True:
#             # data = socket.ReadReceivedData() # read data

#             # if data != None: # if NEW data has been received since last ReadReceivedData function call
#             #     coordinate_parts = data.strip('()').split(',')
#             #     coordinate_list = [float(part) for part in coordinate_parts]

#             #     DroneARCoord = np.array(coordinate_list)

#             #     correctX = -(DroneARCoord[2])
#             #     correctY =  (DroneARCoord[0])
#             #     correctZ =  (DroneARCoord[1])

#             #     print('Drone AR Pos: ({}, {}, {})'.format(correctX, correctY, correctZ))
#             # else:
#             #     print("Not Connected to the Unity Client...  Reconnecting .... ")
            
#             # time.sleep(0.1)
#             DroneARCoord = UDP_warpper.ReadUDPPose()
            
#             correctX = -(DroneARCoord[2])
#             correctY =  (DroneARCoord[0])
#             correctZ =  (DroneARCoord[1])

#             usrInput = 'y' #input('User Command: ')

#             if usrInput == 't':
#                 # print('t')
#                 cf.high_level_commander.takeoff(0.3, flight_time)
#                 time.sleep(2)

#             elif usrInput == 'l':
#                 cf.high_level_commander.land(0.0, flight_time)
#                 time.sleep(2) 

#             elif usrInput == 'y':
#                 cf.high_level_commander.go_to(correctX, correctY, correctZ, 0, flight_time, relative=False)
#                 time.sleep(flight_time) 

#             if usrInput == '.':
#                 print('end of program.. ')
#                 cf.high_level_commander.land(0.0, flight_time)
#                 break

#     UDP_warpper.close()