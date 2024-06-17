# SUCCESSFULLY USE OPTITRACK MOCAP TO FLY TWO CRAZYFLIS!!!! 
# PC Input TWO Drones  (With Full Pose from optiTrack!!!)

# Require BCI Testing!!!!!!!!!  (THIS IS NOT TESTED YET) !!!!!!!!!

"""
The layout of the positions:

           Computers 

       -x1     x0      x1
                
               ^ Y  (mocap -z)         (blue points to z (door))
 y1            |
       2nd     2      1st  
               |
 y0            1-------> X  (mocap x) (red points to motion platform)
                
       3rd            4th
-y1

              Door

"""

import time
import math
import sys

# --- BCI --- 
import socket
import struct
# -----------

from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

from scipy.spatial.transform import Rotation
from threading import Thread
import numpy as np

# URI to the Crazyflie to connect to
uri80 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')            # Red   (light guard) (My Drone)
# uri20 = uri_helper.uri_from_env(default='radio://0/40/2M/E7E7E7E7E7')            # Blue (stepped on)   (initially firmware broken but fixed)
uri10 = uri_helper.uri_from_env(default='radio://0/10/2M/E7E7E7E7E7')              # white with protective guard
uri20 = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E7E7')              # First working drone with mocap + BCI (no guard)


# True: send position and orientation;    False: send position only (not stable)
send_full_pose = True

# True: When all the paramaters are set, and ready fro flying;    False: if otherwise
drone10_ready_flag = False
drone20_ready_flag = False

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90 degrees.
# If this is a problem, increase orientation_std_dev a bit. 
orientation_std_dev = 0.0095      # for mocap 0.04        # The default value in the firmware is 4.5e-3.            8.0e-3


### =====================================================================================================================================================
### Start of the CrazyMocap Class =======================================================================================================================
class CrazyMoCap(Thread):
    def __init__(self, drone80, drone20):
        Thread.__init__(self)

        self._stay_open = True
        self.on_pose = None
        self.initial_pose_set1 = False
        self.initial_pose_set2 = False
        
        # Setup global drone position variables
        self.position = [0.0,0.0,0.0]
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.cf10 = drone10
        self.cf20 = drone20

        self.rotation = [0.0,0.0,0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.start()
        print('Thread Started')

        streaming_client = NatNetClient()

        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.new_frame_listener = self.receiveNewFrame
        streaming_client.rigid_body_listener = self.receiveRigidBodyFrame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.

        is_running = streaming_client.run()
        if not is_running:
            print("ERROR: Could not start streaming client.")
            try:
                sys.exit(1)
            except SystemExit:
                print("...")
            finally:
                print("exiting")


    def close(self):
        self._stay_open = False
        self.join()
        print('Thread closed')



    # This is a callback function that gets connected to the NatNet client
    # and called once per mocap frame.
    def receiveNewFrame(self, data_dict):
        order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                    "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
        dump_args = False
        if dump_args == True:
            out_string = "    "
            for key in data_dict:
                out_string += key + "="
                if key in data_dict :
                    out_string += data_dict[key] + " "
                out_string+="/"
            #print(out_string)



    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receiveRigidBodyFrame(self, new_id, position, rotation ):
        pass
        self.position = position
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]

        self.rotation = (rotation[0], rotation[1], rotation[2], rotation[3])
        cal_mocap_rpy = quaternion_to_euler(rotation[1], rotation[2], rotation[3], rotation[0])

        self.on_pose=([position[0],position[1],position[2],cal_mocap_rpy[2]])  # This line NOT NEEDED    
                                                                                                                             
        if new_id == 1:   # Crazy-10
            self.C10x = position[0]
            self.C10y = position[1]
            self.C10z = position[2]

            self.C10Yaw = cal_mocap_rpy[2] # Y
            self.C10qw = rotation[0]
            self.C10qx = rotation[1]
            self.C10qy = rotation[2]
            self.C10qz = rotation[3]

            if self.initial_pose_set1 == False:
                set_initial_position(self.cf10, self.C10x, self.C10y, self.C10z, self.C10Yaw)
                self.initial_pose_set1 = True    

            # print("Mocap CF10 --->  x: " + str(self.C10x) + "  y: " + str(self.C10y) + "  z: " + str(self.C10z))
            send_extpose_rot_matrix(self.cf10, self.C10x, self.C10y, self.C10z, self.C10qw, self.C10qx, self.C10qy, self.C10qz)   # (drone, x, y, z, roll, pitch, yaw)  

            # self.C10Pose = {'x': self.C10x, 'y': self.C10y,'z': self.C10z, 'yaw': self.C10Yaw, 'qw': self.C10qw, 'qx': self.C10qx, 'qy': self.C10qy, 'qz': self.C10qz}

        if new_id == 2:   # Crazy-20
            self.C20x = position[0]
            self.C20y = position[1]
            self.C20z = position[2]

            self.C20Yaw = cal_mocap_rpy[2] # Y
            self.C20qw = rotation[0]
            self.C20qx = rotation[1]
            self.C20qy = rotation[2]
            self.C20qz = rotation[3]

            if self.initial_pose_set2 == False:
                set_initial_position(self.cf20, self.C20x, self.C20y, self.C20z, self.C20Yaw)
                self.initial_pose_set2 = True  

            # print("Mocap CF20 ---->  x: " + str(self.C20x) + "  y: " + str(self.C20y) + "  z: " + str(self.C20z))
            send_extpose_rot_matrix(self.cf20, self.C20x, self.C20y, self.C20z, self.C20qw, self.C20qx, self.C20qy, self.C20qz)   # (drone, x, y, z, roll, pitch, yaw)
      
        # time.sleep(0.005)                  
        
        # print("x: " + str(position[0]) + "  y: " + str(position[1]) + "  z: " + str(position[2]))
        # print("yaw: " + str(rotation[2]) + "  pitch: " + str(rotation[0]) + "  roll: " + str(rotation[1]))
        # print("DEG: yaw: " + str(math.degrees(rotation[2])) + "  pitch: " + str(math.degrees(rotation[0])) + "  roll: " + str(math.degrees(rotation[1])))
        

    def my_parse_args(arg_list, args_dict):
        # set up base values
        arg_list_len=len(arg_list)
        if arg_list_len>1:
            args_dict["serverAddress"] = arg_list[1]
            if arg_list_len>2:
                args_dict["clientAddress"] = arg_list[2]
            if arg_list_len>3:
                if len(arg_list[3]):
                    args_dict["use_multicast"] = True
                    if arg_list[3][0].upper() == "U":
                        args_dict["use_multicast"] = False

        return args_dict 
    
### End of the CrazyMocap Class =========================================================================================================================
### =====================================================================================================================================================




# Position Estimator functions ==========================================================================================================================
def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=50)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break
            # print(data)


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)
    print('Resetting Position Estimator..')

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


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles to quaternion.

    Args:
        roll: float, rotation around x-axis in radians
        pitch: float, rotation around y-axis in radians
        yaw: float, rotation around z-axis in radians

    Returns:
        numpy.ndarray, quaternion (w, x, y, z) where w is the scalar component
        and x, y, z are the vector components.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return np.array([w, x, y, z])


def normalise_quaternion(q):
    norm = np.linalg.norm(q)
    if norm == 0:
        return np.array([1,0,0,0])
    else:
        return q / norm

# =======================================================================================================================================================




# A Function to take the OptiTrack real-time pose data and send them to Crazyflie =======================================================================
def send_extpose_rot_matrix(cf, x, y, z, qx, qy, qz, qw):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
    rotaton matrix. This is going to be forwarded to the Crazyflie's
    position estimator.
    """

    # print('Sending Expos to Drone!! ')
    
    rounded_x =    x      #round(x, 5)              # -round(y, 5)
    rounded_y =    y      #round(y, 5)              #  round(x, 5)
    rounded_z =    z-0.04 #round(z-0.02, 5)

    # print('Mocap -------  x: ' + str(rounded_x) + '  y: ' + str(rounded_y) + '  z: ' + str(rounded_z))
    # print('Mocap -------  qx: ' + str(norm_rot[1]) + '  qy: ' + str(norm_rot[2]) + '  qz: ' + str(norm_rot[3])  + '  qw: ' + str(norm_rot[0]))
    
    if (drone10_ready_flag == True and drone20_ready_flag == True):
        if send_full_pose:
            cf.extpos.send_extpose(rounded_x, rounded_y, rounded_z, qx, qy, qz, qw)
            # print("mocap drone: " + str(cf.link_uri) + "  pos is being set for : {}, {}, {}".format(rounded_x, rounded_y, rounded_z))
        else:
            cf.extpos.send_extpos(rounded_x, rounded_y, rounded_z)


def set_initial_position(drone, x, y, z, yaw_deg):
    drone.param.set_value('kalman.initialX', x)
    drone.param.set_value('kalman.initialY', y)
    drone.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    drone.param.set_value('kalman.initialYaw', yaw_radians)
    print('setting initial position..')

# =======================================================================================================================================================




# A callback function for displaying real time position (xyz) data of a specific drone ==================================================================
def position_callback(uri, timestamp, data, logconf):       
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print("{} is at pos: ({}, {}, {})".format(uri, x, y, z))        # can print
    
def start_position_printing(drone):
    log_conf = LogConfig(name='KPosition', period_in_ms=100)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    uri = drone.link_uri
    drone.log.add_config(log_conf)
    # log_conf.data_received_cb.add_callback(position_callback)
    log_conf.data_received_cb.add_callback(lambda timestamp, data, logconf: position_callback(uri, timestamp, data, logconf) )
    log_conf.start()


# A callback function for displaying real time orientation (roll, pitch, yaw) data of a specific drone -------------------------------------------------
def orientation_callback(timestamp, data, logconf):
    qw = data['kalman.q0']   # qw
    qx = data['kalman.q1']   # qx
    qy = data['kalman.q2']   # qy
    qz = data['kalman.q3']   # qz

    eular = quaternion_to_euler(qw, qx, qy, qz)

    print('Drone Orient DEG: ( Roll: {}, Pitch: {}, Yaw: {})'.format(math.degrees(eular[0]), math.degrees(eular[1]), math.degrees(eular[2])))
    # print('Drone Orient QUAT: ({}, {}, {}, {})'.format(qx, qy, qz, qw))

def start_orientation_printing(drone):
    log_conf = LogConfig(name='Orientation', period_in_ms=50)
    log_conf.add_variable('kalman.q0', 'float')   # qw
    log_conf.add_variable('kalman.q1', 'float')   # qx
    log_conf.add_variable('kalman.q2', 'float')   # qy
    log_conf.add_variable('kalman.q3', 'float')   # qz

    drone.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(orientation_callback)
    log_conf.start()
    

# A callback function for displaying real time Motor PWM value of all four motors ---------------------------------------------------------------------
def motor_callback(timestamp, data, logconf):
    m1 = data['motor.m1']
    m2 = data['motor.m2']
    m3 = data['motor.m3']
    m4 = data['motor.m4']
    print('motor: ({}, {}, {}, {})'.format(m1, m2, m3, m4))

def start_motor_printing(drone):
    log_conf = LogConfig(name='Position', period_in_ms=50)
    log_conf.add_variable('motor.m1', 'float')
    log_conf.add_variable('motor.m2', 'float')
    log_conf.add_variable('motor.m3', 'float')
    log_conf.add_variable('motor.m4', 'float')

    drone.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(motor_callback)
    log_conf.start()

# =======================================================================================================================================================





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

# ============================================================================




# Activate various settings and parameters of the drone's internal calculation approachs and controllers ================================================
def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)   # 0.04
    print('Adjusting orientation Sensitivity..')


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    print('Activate Kalman Estimator..')


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')
    print('Activate High Level Commander..')


def activate_mellinger_controller(cf):                  # Not currently in use!!!           # The working version dont have mellinger enabled!!!
    cf.param.set_value('stabilizer.controller', '2')    # 1 for PID and 2 for Mellinger and 3 for INDI
    print('Activate Mellinger Controller..')

# =======================================================================================================================================================




# A Function that takes the user keyboard input t,l,0,1,2,3,4,5,., and send the respective flight command to the drone ==================================
def user_input(cf10, cf20, range, flight_time):   

    print('----- Accepting User Keyboard Input -----')

    differences = 0.4
    relative = False      # Use absolute for a single drone (relative = False)

    while True:
        
        usrInput = input('User Command: ')

        if usrInput == 't':
            
            # Take Off
            cf10.high_level_commander.takeoff(0.3, 3)
            cf20.high_level_commander.takeoff(0.3, 3)
            print('taking off!')
            time.sleep(2)

        elif usrInput == 'l':

            # Landing
            cf10.high_level_commander.land(0.01, 3)
            cf20.high_level_commander.land(0.01, 3)
            print('Landing! ')
            time.sleep(2)  

        elif usrInput == '0':

            # Position 0    Elevate to 0.6m from ground
            cf10.high_level_commander.go_to(0, 0, 0.6, 0, flight_time, relative)
            cf20.high_level_commander.go_to(0, 0+differences, 0.6, 0, flight_time, relative)
            print('Pos 0')
            time.sleep(flight_time)

        elif usrInput == '1':

            # Position 1  First Quad   
            cf10.high_level_commander.go_to(range, range, 0.6, 0, flight_time, relative)
            cf20.high_level_commander.go_to(range, range+differences, 0.6, 0, flight_time, relative)
            print('Pos 1')
            time.sleep(flight_time)

        elif usrInput == '2':

            # Position 2   Second Quad
            cf10.high_level_commander.go_to(-range, range, 0.6, 0, flight_time, relative)
            cf20.high_level_commander.go_to(-range, range+differences, 0.6, 0, flight_time, relative)
            print('Pos 2')
            time.sleep(flight_time)

        elif usrInput == '3':

            # Position 3   Third Quad
            cf10.high_level_commander.go_to(-range, -range, 0.6, 0, flight_time, relative)
            cf20.high_level_commander.go_to(-range, -range+differences, 0.6, 0, flight_time, relative)
            print('Pos 3')
            time.sleep(flight_time)

        elif usrInput == '4':

            # Position 4    Fourth Quad
            cf10.high_level_commander.go_to(range, -range, 0.6, 0, flight_time, relative)
            cf20.high_level_commander.go_to(range, -range+differences, 0.6, 0, flight_time, relative)            
            print('Pos 4')
            time.sleep(flight_time)

        elif usrInput == '5':

            # Position 5    Lower for Landing 
            cf10.high_level_commander.go_to(0, 0, 0.2, 0, flight_time, relative)
            cf20.high_level_commander.go_to(0, 0+differences, 0.2, 0, flight_time, relative)
            print('Pos 5')
            time.sleep(flight_time)

        if usrInput == '.':

            # Exiting Function
            cf10.high_level_commander.land(0.0, 1)
            cf20.high_level_commander.land(0.0, 1)
            time.sleep(1)
            print('Exiting User Input... ')
            break

# =======================================================================================================================================================




# The Main Function of the MocapSwarmUserInput.py ==========================================================================================================
if __name__ == '__main__':
    # Instantiate the Crazyflie Driver
    cflib.crtp.init_drivers()
    
    try:
        with SyncCrazyflie(uri10, cf=Crazyflie(rw_cache='./cache')) as scf10:

            with SyncCrazyflie(uri20, cf=Crazyflie(rw_cache='./cache')) as scf20:

                # Create a single drone object
                drone10 = scf10.cf
                drone20 = scf20.cf  

                # Connect to OptiTrack 
                crazymocap_warpper = CrazyMoCap(drone10,drone20)    

                # start_position_printing(drone10)
                # start_position_printing(drone20)

                # start_orientation_printing(drone10)
                # start_orientation_printing(drone20)

                # start_motor_printing(drone10)
                # start_motor_printing(drone20)

                print('\n============= SETTING UP DRONES =============')
                adjust_orientation_sensitivity(drone10)
                adjust_orientation_sensitivity(drone20)

                activate_kalman_estimator(drone10)
                activate_kalman_estimator(drone20)

                activate_high_level_commander(drone10)
                activate_high_level_commander(drone20)

                activate_mellinger_controller(drone10)        # Working version DO NOT have this enabled! (use PID)
                activate_mellinger_controller(drone20)

                drone10_ready_flag = True
                drone20_ready_flag = True

                reset_estimator(drone10)                   #Need!
                reset_estimator(drone20)  
                print('=============================================\n')

                print('========== SETTING UP BCI ==============')
                my_ip=socket.gethostbyname(socket.gethostname())
                # initailize receiving socket
                UDP_PORT = 5005
                UDP_IP = my_ip
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
                sock.setblocking(False)
                sock.bind((UDP_IP, UDP_PORT))
                feed_back_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                feed_back_PORT = 5006
                # feed_back_IP = "192.168.0.10" # control PC IP
                feed_back_IP = my_ip
                print('========================================')

                # Any Control script here -------------------------------------------------------------------------------------
                differences = 0.4  # 0.4 m between each drones
                relative = False   # Absolute
                flight_time = 3    # 3 seconds

                drone10.high_level_commander.takeoff(0.3, 3)
                drone20.high_level_commander.takeoff(0.3, 3)
                print('taking off!')
                time.sleep(2)

                while True:
                    print('----- Start Listening Commands -----')

                    BRI_command = receive_command(sock)

                    if BRI_command == 1 :
                        # Position 1  First Quad   
                        drone10.high_level_commander.go_to(range, range, 0.6, 0, flight_time, relative)
                        drone20.high_level_commander.go_to(range, range+differences, 0.6, 0, flight_time, relative)
                        print('Pos 1')
                        time.sleep(flight_time)
                        send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                    
                    elif BRI_command == 2 :
                        # Position 2   Second Quad
                        drone10.high_level_commander.go_to(-range, range, 0.6, 0, flight_time, relative)
                        drone20.high_level_commander.go_to(-range, range+differences, 0.6, 0, flight_time, relative)
                        print('Pos 2')
                        time.sleep(flight_time)
                        send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                    
                    elif BRI_command == 3 :
                        # Position 3   Third Quad
                        drone10.high_level_commander.go_to(-range, -range, 0.6, 0, flight_time, relative)
                        drone20.high_level_commander.go_to(-range, -range+differences, 0.6, 0, flight_time, relative)
                        print('Pos 3')
                        time.sleep(flight_time)
                        send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                    
                    elif BRI_command == 4 :
                        # Position 4    Fourth Quad
                        drone10.high_level_commander.go_to(range, -range, 0.6, 0, flight_time, relative)
                        drone20.high_level_commander.go_to(range, -range+differences, 0.6, 0, flight_time, relative)            
                        print('Pos 4')
                        time.sleep(flight_time)
                        send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                    
                    elif BRI_command == 5 :
                        # Position High    Elevate to 0.6m from ground
                        drone10.high_level_commander.go_to(0, 0, 0.6, 0, flight_time, relative)
                        drone20.high_level_commander.go_to(0, 0+differences, 0.6, 0, flight_time, relative)
                        print('High')
                        time.sleep(flight_time)
                        send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                    
                    elif BRI_command == 6 :
                        # Position Low    Lower for Landing 
                        drone10.high_level_commander.go_to(0, 0, 0.2, 0, flight_time, relative)
                        drone20.high_level_commander.go_to(0, 0+differences, 0.2, 0, flight_time, relative)
                        print('Low')
                        time.sleep(flight_time)
                        send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

                    elif BRI_command == -1 :
                        drone10.high_level_commander.land(0.01, 3)
                        drone20.high_level_commander.land(0.01, 3)
                        print('Landing! ')
                        send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                        break
                # ---------------------------------
            

        crazymocap_warpper.close()
        scf10.close_link()
        scf20.close_link()

        print('Sequence Done!  Closing Link...')

    except KeyboardInterrupt:
        print("Keyboard Interruptted!  Closing Link.... ")

        # drone.high_level_commander.land(0.00, 0.5)
        print('Emergency Landing!! ')
        scf10.close_link()
        scf20.close_link()
    

