# ## Keyboard Controlled Central Drone (Leader) with two Follower Drone (left and right)
# 17/05/2023
# NOT COMPLETE YET!!!!!!

# Optimise: 
#   1. 
#   2. 
#   3. Need testing with BCI HoloLens!!!


import time
import math
from threading import Thread

# ---- Keyboard ---
import logging
from pynput import keyboard
# -----------------

# --- BCI --- 
import socket
import struct
# -----------

import cflib.crtp
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.utils import uri_helper


rangee = 0.75
flight_time = 2
relative = False
relativee = True

h = 0.5  # remain constant height similar to take off height


xL, yL = 0.0, +0.4   # Left
xR, yR = 0.0, -0.4   # Right


# Home Testing Drone URLs
uris = [
    'radio://0/40/2M/E7E7E7E7E7',  # cf_id 0, startup position [ 0, +0.4 ]     (Left)
    'radio://0/30/2M/E7E7E7E7E7',  # cf_id 1, startup position [ 0,  0.0 ]     (with light)
    'radio://0/80/2M/E7E7E7E7E7',  # cf_id 2, startup position [ 0, -0.4 ]     (Right)
    # Add more URIs if you want more copters in the swarm 
]


# URI Address for the Leader Drone
LeaderDrone = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E7')     # (Middle)
FollowerLeft = uri_helper.uri_from_env(default='radio://0/40/2M/E7E7E7E7E7')    # (Left)
FollowerRight = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')   # (Right)


# =========================== BCI ============================================ (Not Used)

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

# =========================== Keyboard Control ===========================================

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class KeyboardDrone(Thread):

    def __init__(self, mc):
        Thread.__init__(self)
        self.mc = mc

        self.velocity = 0.75
        self.ang_velocity = 120

        self.sleeptime = 0.5
        # self.max_hight = 0.8
        # self.hight = 0.0

        self.LeaderTakeOff = False
        self.LeaderLanding = False
        self.killed = False

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.start()

        print('---------Thread Started----------')
        print('Press t to Take Off!')
    
    def getTakeoffStatus(self):
        return self.LeaderTakeOff
    
    def getLandingStatus(self):
        return self.LeaderLanding

    def on_press(self, key):

        if key == keyboard.Key.space:
            print('Starting Keyboard Control!')

        if key.char == 'w':
            self.mc.start_forward(self.velocity)
            print('Forward')

        if key.char == 't':
            self.mc.take_off(h)
            self.LeaderTakeOff = True
            self.LeaderLanding = False
            print('Take off!')

        if key.char == 'l':
            self.mc.land()
            self.LeaderLanding = True
            self.LeaderTakeOff = False
            print('Landing')

        if key.char == 's':
            self.mc.start_back(self.velocity)
            print('Backward')

        if key.char == 'a':
            self.mc.start_left(self.velocity)
            print('Left')

        if key.char == 'd':
            self.mc.start_right(self.velocity)
            print('Right')

        if key.char == ',':
            self.mc.start_down(self.velocity)
            print('Decending')

        if key.char == '.':
            self.mc.start_up(self.velocity)
            print('Ascending')

        if key.char == 'k':
            print('Kill Motor')
            self.killed = True
            return False

        if key.char == 'q':
            self.mc.start_turn_left(self.ang_velocity)
            print('Rotate Left')

        if key.char == 'e':
            self.mc.start_turn_right(self.ang_velocity)
            print('Rotate Right')


    def on_release(self, key):
        self.mc.stop()
    
    def close(self):
        self.join()
        print('Thread closed')


# ============================ Drone Control ====================================================================================================================

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
    # print('Drone Leader: ({},{},{})'.format(x, y, z)) 
    global leaderPos
    leaderPos = [x, y, z]

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

    # time.sleep(0.01)            # <--------- Delay the clearing 
    # print("\033c", end='')      # <--------- Put this at the end of drone to clear console!!!!!!!!!
    

def start_position_printing(scf):
    print('Start position printing: ')

    if scf.cf.link_uri == 'radio://0/30/2M/E7E7E7E7E7':                               #  <--------------------- Change URI Here
           
        log_conf = LogConfig(name='Position', period_in_ms=100)
        log_conf.add_variable('kalman.stateX', 'float')
        log_conf.add_variable('kalman.stateY', 'float')
        log_conf.add_variable('kalman.stateZ', 'float')

        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(position_callback_centre)
        log_conf.start()

        

    # ## Add more uris to print different position coordinates for more drosnes
    # if scf.cf.link_uri == 'radio://0/40/2M/E7E7E7E7E7':                               #  <--------------------- Change URI Here
         
    #     log_conf = LogConfig(name='Position', period_in_ms=100)
    #     log_conf.add_variable('kalman.stateX', 'float')
    #     log_conf.add_variable('kalman.stateY', 'float')
    #     log_conf.add_variable('kalman.stateZ', 'float')

    #     scf.cf.log.add_config(log_conf)
    #     log_conf.data_received_cb.add_callback(position_callback_L)
    #     log_conf.start()


    # if scf.cf.link_uri == 'radio://0/80/2M/E7E7E7E7E7':                               #  <--------------------- Change URI Here
         
    #     log_conf = LogConfig(name='Position', period_in_ms=100)
    #     log_conf.add_variable('kalman.stateX', 'float')
    #     log_conf.add_variable('kalman.stateY', 'float')
    #     log_conf.add_variable('kalman.stateZ', 'float')

    #     scf.cf.log.add_config(log_conf)
    #     log_conf.data_received_cb.add_callback(position_callback_R)
    #     log_conf.start()

    


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


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')
    print('Activate High Level Commander..')

def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)

# =======================================================================================================================================================

if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_debug_driver=False)
    global leaderPos
    leaderPos = [0.0, 0.0, 0.0]

    try:
        with SyncCrazyflie(LeaderDrone) as leader:
            mc = MotionCommander(leader)
            leaderdrone = KeyboardDrone(mc)

            # with keyboard.Listener(on_press=leaderdrone.on_press, on_release=leaderdrone.on_release) as listener:
            #     listener.join()
            #     start_position_printing(leader)

            # Initial the Follower Drones: 
            with SyncCrazyflie(FollowerLeft) as FLeft:
                with SyncCrazyflie(FollowerRight) as FRight:

                    with keyboard.Listener(on_press=leaderdrone.on_press, on_release=leaderdrone.on_release) as listener:
                        
                        # Create a single drone object
                        left = FLeft.cf
                        right = FRight.cf

                        activate_high_level_commander(left)
                        activate_high_level_commander(right)

                        start_position_printing(leader)

                        while True: 
                            # print("MAIN: ", leaderPos)

                            if leaderdrone.LeaderTakeOff == True and leaderdrone.LeaderLanding == False: 
                                left.high_level_commander.takeoff(h, 3)
                                right.high_level_commander.takeoff(h, 3)

                                left.high_level_commander.go_to(leaderPos[0], leaderPos[1]-0.5, leaderPos[2], flight_time, relative)
                                right.high_level_commander.go_to(leaderPos[0], leaderPos[1]+0.5, leaderPos[2], flight_time, relative)

                            if leaderdrone.LeaderTakeOff == False and leaderdrone.LeaderLanding == True: 
                                left.high_level_commander.land(0.00, 3)
                                right.high_level_commander.land(0.00, 3)

                            if leaderdrone.killed == True:
                                print('Leader Killed, Followers Landing!')
                                left.high_level_commander.land(0.00, 0.5)
                                right.high_level_commander.land(0.00, 0.5)
                                break
                        

    except KeyboardInterrupt:
        print("Keyboard Interruptted!  Closing Link.... ")
        # drone.high_level_commander.land(0.00, 0.5)
        print('Emergency Landing!! ')
        leader.close_link()
        FLeft.close_link()
        FRight.close_link()