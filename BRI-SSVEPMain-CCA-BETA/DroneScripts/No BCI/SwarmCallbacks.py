# ## Keyboard Controlled Central Drone (Leader) with two Follower Drone (left and right)
# 17/05/2023
# NOT COMPLETE YET!!!!!!

# Optimise: 
#   1. 
#   2. 
#   3. Need testing with BCI HoloLens!!!


import time
import math

# --- BCI --- 
import socket
import struct
# -----------

import cflib.crtp
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


rangee = 0.75
flight_time = 2
relative = False
relativee = True

h = 0.5  # remain constant height similar to take off height

xL, yL = 0.0, +0.4   # Left
xR, yR = 0.0, -0.4   # Right


# uris = [
#     'radio://0/40/2M/E7E7E7E7E7',  # cf_id 0, startup position [ 0, +0.4 ]     (Left)
#     'radio://0/30/2M/E7E7E7E7E7',  # cf_id 1, startup position [ 0,  0.0 ]     (with light)
#     'radio://0/80/2M/E7E7E7E7E7',  # cf_id 2, startup position [ 0, -0.4 ]     (Right)
#     # Add more URIs if you want more copters in the swarm 
# ]


uris = [
    'radio://0/10/2M/E7E7E7E7E7',  # cf_id 0, startup position [ 0, +0.4 ]     (Left)
    'radio://0/20/2M/E7E7E7E7E7',  # cf_id 1, startup position [ 0,  0.0 ]     (with light)
    'radio://0/39/2M/E7E7E7E7E7',  # cf_id 2, startup position [ 0, -0.4 ]     (Right)
    # Add more URIs if you want more copters in the swarm 
]

# droneLeft = [  # Crazy-10
#      (xL, yL, h+0.4, flight_time),      # Highest 7
#      (xL, yL, h-0.1, flight_time),      # Lowest  8
#      (xL-rangee, yL+rangee, h, flight_time),        # Pos1
#      (xL-rangee, yL       , h, flight_time),        # Pos2
#      (xL-rangee, yL-rangee, h, flight_time),        # Pos3
#      (xL+rangee, yL+rangee, h+h, flight_time),      # Pos4
#      (xL+rangee, yL       , h+h, flight_time),      # Pos5
#      (xL+rangee, yL-rangee, h+h, flight_time),      # Pos6

# ]

# droneMiddle = [  # Crazy-30 (Light)
#      (0, 0, h+0.6, flight_time),      # Highest 7
#      (0, 0, h-0.1, flight_time),      # Lowest  8
#      (-rangee,  rangee, h+0.2, flight_time),        # Pos1
#      (-rangee, 0      , h+0.2, flight_time),        # Pos2
#      (-rangee, -rangee, h+0.2, flight_time),        # Pos3
#      ( rangee,  rangee, h+h+0.2, flight_time),      # Pos4
#      ( rangee, 0      , h+h+0.2, flight_time),      # Pos5
#      ( rangee, -rangee, h+h+0.2, flight_time),      # Pos6

# ]

# droneRight = [   # Crazy-80
#      (xR, yR, h+0.4, flight_time),      # Highest 7
#      (xR, yR, h-0.1, flight_time),      # Lowest  8
#      (xR-rangee, yR+rangee, h, flight_time),        # Pos1
#      (xR-rangee, yR       , h, flight_time),        # Pos2
#      (xR-rangee, yR-rangee, h, flight_time),        # Pos3
#      (xR+rangee, yR+rangee, h+h, flight_time),      # Pos4
#      (xR+rangee, yR       , h+h, flight_time),      # Pos5
#      (xR+rangee, yR-rangee, h+h, flight_time),      # Pos6
# ]


# seq_args = {
#     uris[0]: [droneLeft],
#     uris[1]: [droneMiddle],
#     uris[2]: [droneRight],
#     # uris[3]: [sequence3],
# }


# =========================== BCI ============================================


def send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT):
    # print("sending feedback")
    MESSAGE = struct.pack('!i', 0)
    feed_back_sock.sendto(MESSAGE, (feed_back_IP, feed_back_PORT))


def receive_command(udp_socket):
    try:
        b_data, addr = udp_socket.recvfrom(1024)  # buffer size is 1024 bytes
        data = struct.unpack('!i', b_data)[0]
        print("receive target: %s" % data)
        target = data
        return data

    except:
        return None


# =======================================================================================================================================================
# Math Functions to convert between Euler and Quaternion Orientations ===================================================================================
def quaternion_to_euler(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z     # in radians


# A callback function for displaying real time position (xyz) data of a specific drone ==================================================================
def position_callback_centre(timestamp, data, logconf):

    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('Drone Leader: ({},{},{})'.format(x, y, z))    

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

    if scf.cf.link_uri == 'radio://0/10/2M/E7E7E7E7E7':                               #  <--------------------- Change URI Here
           
        log_conf = LogConfig(name='Position', period_in_ms=100)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(position_callback_centre)
        log_conf.start()

    ## Add more uris to print different position coordinates for more drosnes
    if scf.cf.link_uri == 'radio://0/20/2M/E7E7E7E7E7':                               #  <--------------------- Change URI Here
         
        log_conf = LogConfig(name='Position', period_in_ms=100)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(position_callback_L)
        log_conf.start()


    if scf.cf.link_uri == 'radio://0/39/2M/E7E7E7E7E7':                               #  <--------------------- Change URI Here
         
        log_conf = LogConfig(name='Position', period_in_ms=100)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(position_callback_R)
        log_conf.start()

    


# A callback function for displaying real time orientation (roll, pitch, yaw) data of a specific drone -------------------------------------------------
def orientation_callback_centre(timestamp, data, logconf):
    qw = data['kalman.q0']   # qw
    qx = data['kalman.q1']   # qx
    qy = data['kalman.q2']   # qy
    qz = data['kalman.q3']   # qz

    eular = quaternion_to_euler(qw, qx, qy, qz)

    print('Leader Drone: ( Roll: {}, Pitch: {}, Yaw: {})'.format(math.degrees(eular[0]), math.degrees(eular[1]), math.degrees(eular[2])))
    # print('Leader Drone Orient QUAT: ({}, {}, {}, {})'.format(qx, qy, qz, qw))

def orientation_callback_L(timestamp, data, logconf):
    qw = data['kalman.q0']   # qw
    qx = data['kalman.q1']   # qx
    qy = data['kalman.q2']   # qy
    qz = data['kalman.q3']   # qz

    eular = quaternion_to_euler(qw, qx, qy, qz)

    print('Follower Drone (L): ( Roll: {}, Pitch: {}, Yaw: {})'.format(math.degrees(eular[0]), math.degrees(eular[1]), math.degrees(eular[2])))
    # print('Leader Drone Orient QUAT: ({}, {}, {}, {})'.format(qx, qy, qz, qw))

def orientation_callback_R(timestamp, data, logconf):
    qw = data['kalman.q0']   # qw
    qx = data['kalman.q1']   # qx
    qy = data['kalman.q2']   # qy
    qz = data['kalman.q3']   # qz

    eular = quaternion_to_euler(qw, qx, qy, qz)

    print('Follower Drone (R): ( Roll: {}, Pitch: {}, Yaw: {})'.format(math.degrees(eular[0]), math.degrees(eular[1]), math.degrees(eular[2])))
    # print('Leader Drone Orient QUAT: ({}, {}, {}, {})'.format(qx, qy, qz, qw))
    
    time.sleep(0.01)            # <--------- Delay the clearing 
    print("\033c", end='')      # <--------- Put this at the end of drone to clear console!!!!!!!!!


def start_orientation_printing(scf):
    if scf.cf.link_uri == 'radio://0/10/2M/E7E7E7E7E7':                               #  <--------------------- Change URI Here
        log_conf = LogConfig(name='Orientation', period_in_ms=100)
        log_conf.add_variable('kalman.q0', 'float')   # qw
        log_conf.add_variable('kalman.q1', 'float')   # qx
        log_conf.add_variable('kalman.q2', 'float')   # qy
        log_conf.add_variable('kalman.q3', 'float')   # qz

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(orientation_callback_centre)
        log_conf.start()

    if scf.cf.link_uri == 'radio://0/20/2M/E7E7E7E7E7':                               #  <--------------------- Change URI Here
        log_conf = LogConfig(name='Orientation', period_in_ms=100)
        log_conf.add_variable('kalman.q0', 'float')   # qw
        log_conf.add_variable('kalman.q1', 'float')   # qx
        log_conf.add_variable('kalman.q2', 'float')   # qy
        log_conf.add_variable('kalman.q3', 'float')   # qz

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(orientation_callback_L)
        log_conf.start()

    if scf.cf.link_uri == 'radio://0/39/2M/E7E7E7E7E7':                               #  <--------------------- Change URI Here
        log_conf = LogConfig(name='Orientation', period_in_ms=100)
        log_conf.add_variable('kalman.q0', 'float')   # qw
        log_conf.add_variable('kalman.q1', 'float')   # qx
        log_conf.add_variable('kalman.q2', 'float')   # qy
        log_conf.add_variable('kalman.q3', 'float')   # qz

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(orientation_callback_R)
        log_conf.start()
    

# =======================================================================================================================================================


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)

# =======================================================================================================================================================


def PosHigh(scf: SyncCrazyflie, Poses): 
    # Position High   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[0][0], Poses[0][1], Poses[0][2]
    duration = Poses[0][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Highest Middle')
    time.sleep(flight_time)


def PosLow(scf: SyncCrazyflie, Poses): 
    # Position Low    
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[1][0], Poses[1][1], Poses[1][2]
    duration = Poses[1][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Lowest Middle')
    time.sleep(flight_time)


def Pos1(scf: SyncCrazyflie, Poses): 
    # Position 1  First Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[2][0], Poses[2][1], Poses[2][2]
    duration = Poses[2][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 1')
    time.sleep(flight_time)


def Pos2(scf: SyncCrazyflie, Poses): 
    # Position 2  Second Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[3][0], Poses[3][1], Poses[3][2]
    duration = Poses[3][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 2')
    time.sleep(flight_time)


def Pos3(scf: SyncCrazyflie, Poses): 
    # Position 3  Third Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[4][0], Poses[4][1], Poses[4][2]
    duration = Poses[4][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 3')
    time.sleep(flight_time)


def Pos4(scf: SyncCrazyflie, Poses): 
    # Position 4  Fourth Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[5][0], Poses[5][1], Poses[5][2]
    duration = Poses[5][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 4')
    time.sleep(flight_time)


def Pos5(scf: SyncCrazyflie, Poses): 
    # Position 4  Fourth Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[6][0], Poses[6][1], Poses[6][2]
    duration = Poses[6][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 5')
    time.sleep(flight_time)    


def Pos6(scf: SyncCrazyflie, Poses): 
    # Position 4  Fourth Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[7][0], Poses[7][1], Poses[7][2]
    duration = Poses[7][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 6')
    time.sleep(flight_time)


def take_off(scf): 
    commander = scf.cf.high_level_commander
    commander.takeoff(0.5, flight_time)
    time.sleep(2)

def landing(scf): 
    commander = scf.cf.high_level_commander
    commander.land(0.0, flight_time)
    time.sleep(2)   

def higher(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(0.0, 0.0, rangee, 0, flight_time, relativee)   # Forward 0.2m
    print('Higher')
    time.sleep(2)     

def lower(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(0.0, 0.0, -rangee, 0, flight_time, relativee)   # Forward 0.2m
    print('Lower')
    time.sleep(2)     

def left(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(0.0, rangee, 0.0, 0, flight_time, relativee)   # Forward 0.2m
    print('Left')
    time.sleep(2) 

def right(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(0.0, -rangee, 0.0, 0, flight_time, relativee)   # Forward 0.2m
    print('Right')
    time.sleep(2)  

def forward(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(rangee, 0.0, 0.0, 0, flight_time, relativee)   # Forward 0.2m
    print('forward')
    time.sleep(2)  

def backward(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(-rangee, 0.0, 0.0, 0, flight_time, relativee)   # Forward 0.2m
    print('backward')
    time.sleep(2)    


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:

        swarm.parallel_safe(activate_high_level_commander)
        print(swarm.get_estimated_positions())
        swarm.reset_estimators()

        swarm.parallel_safe(start_position_printing)
        # swarm.parallel_safe(start_orientation_printing)

        # Mock User Input :  -------------------------------
        while True:
            BRI_command = input('User Command: ')

            if BRI_command == '1':
                print('1')

            elif BRI_command == '2':
                print('2')   

            elif BRI_command == '3':
                swarm.close_links()
                print('end of program.. ')
                break 


        # print('========== SETTING UP BCI ==============')
        # my_ip=socket.gethostbyname(socket.gethostname())
        # # initailize receiving socket
        # UDP_PORT = 5005
        # UDP_IP = my_ip
        # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        # sock.setblocking(False)
        # sock.bind((UDP_IP, UDP_PORT))
        # feed_back_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # feed_back_PORT = 5006
        # feed_back_IP = my_ip
        # print('my IP: ' + str(my_ip))
        # print('========================================')

        # print('All Ready To Go!')

        # ### swarm.parallel_safe(take_off)   <-- dont use auto take off

        # print('----- Start Listening Commands -----')

        # while True:

        #     # BRI_command = receive_command(sock)
        #     BRI_command = input('User Command: ')

        #     if BRI_command == 1 :
        #         swarm.parallel_safe(Pos1, args_dict=seq_args)
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #         print('1')

        #     elif BRI_command == 2 :
        #         swarm.parallel_safe(Pos2, args_dict=seq_args)
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #         print('2')

        #     elif BRI_command == 3 :
        #         swarm.parallel_safe(Pos3, args_dict=seq_args)
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #         print('3')

        #     elif BRI_command == 4 :
        #         swarm.parallel_safe(Pos4, args_dict=seq_args)
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #         print('4')

        #     elif BRI_command == 5 :
        #         swarm.parallel_safe(Pos5, args_dict=seq_args)
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #         print('5')

        #     elif BRI_command == 6 :     
        #         swarm.parallel_safe(Pos6, args_dict=seq_args)
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #         print('6')

        #     elif BRI_command == 7 :     # Middle High Position 
        #         swarm.parallel_safe(PosHigh, args_dict=seq_args)
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #         print('7')

        #     elif BRI_command == 8 :     # Middle Low Position (prepare for landing)
        #         swarm.parallel_safe(PosLow, args_dict=seq_args)
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #         print('8') 

        #     elif BRI_command == 0 :     # When BCI got an error input, return to the middle 
        #         swarm.parallel_safe(PosLow, args_dict=seq_args)
        #         print('Wrong Signal - Back to Origin')
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

        #     elif BRI_command == 9 :
        #         swarm.parallel_safe(landing)
        #         print('LANDING')
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)    

        #     elif BRI_command == -1 :
        #         swarm.parallel_safe(landing)
        #         swarm.close_links()
        #         print('End of Program.. ')
        #         break
