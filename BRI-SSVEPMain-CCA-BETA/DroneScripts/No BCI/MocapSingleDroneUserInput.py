# SUCCESSFULLY USE OPTITRACK MOCAP TO FLY CRAZYFLIE!!!! 

# Working with mocap and BCI 09/03/2023 ---------------------------------------------------------------------------
# Single Drone Only.  (With Full Pose from optiTrack!!!)
# 09/March/2023 !!!!!!!

# Cleaned and Added Comments and Section lines  - (11/03/2023) 

# changelog: 
    ## orientation_std_dev = 0.0095             in orientation_sensitivity
    ## time.sleep is 0.01                       in send_extpose
    ## NO activate_mellinger_controller(drone)  in main

import time
import math
import sys

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
# uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')            # Red   (light guard) (My Drone)
# uri = uri_helper.uri_from_env(default='radio://0/40/2M/E7E7E7E7E7')            # Blue (stepped on)   (initially firmware broken but fixed)
uri = uri_helper.uri_from_env(default='radio://0/10/2M/E7E7E7E7E7')              # white with protective guard
# uri = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E7E7')              # First working drone with mocap + BCI (no guard)


# True: send position and orientation;    False: send position only (not stable)
send_full_pose = True

# True: When all the paramaters are set, and ready fro flying;    False: if otherwise
drone_ready_flag = False

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90 degrees.
# If this is a problem, increase orientation_std_dev a bit. 
orientation_std_dev = 0.0095      # for Qualisys mocap 0.04        # The default value in the firmware is 4.5e-3.            8.0e-3


### =====================================================================================================================================================
### Start of the CrazyMocap Class =======================================================================================================================
class CrazyMoCap(Thread):
    def __init__(self, drone):
        Thread.__init__(self)

        self._stay_open = True
        self.on_pose = None
        self.initial_pose_set = False
        
        # Setup global drone position variables
        self.position = [0.0,0.0,0.0]
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.cf = drone

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
                                                                                                                             

        if self.initial_pose_set == False:
            set_initial_position(self.cf,position[0],position[1],position[2],cal_mocap_rpy[2])   #x,y,z,yaw
            self.initial_pose_set = True      

        time.sleep(0.01)                   # MUST HAVE  (0.01 for one drone)
        
        send_extpose_rot_matrix(self.cf, position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], rotation[3])   # (drone, x, y, z, roll, pitch, yaw)
        
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
    
    if drone_ready_flag == True:
        if send_full_pose:
            cf.extpos.send_extpose(rounded_x, rounded_y, rounded_z, qx, qy, qz, qw)
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
def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('Drone Pos: ({}, {}, {})'.format(x, y, z))

def start_position_printing(drone):
    log_conf = LogConfig(name='Position', period_in_ms=50)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    drone.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
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
def user_input(cf, range, flight_time):   

    print('----- Accepting User Keyboard Input -----')

    box_size = range
    relative = False      # Use absolute for a single drone (relative = False)

    while True:
        
        usrInput = input('User Command: ')

        if usrInput == 't':
            
            # Take Off
            cf.high_level_commander.takeoff(0.3, 3)
            print('taking off!')
            time.sleep(2)

        elif usrInput == 'l':

            # Landing
            cf.high_level_commander.land(0.01, 3)
            print('Landing! ')
            time.sleep(2)  

        elif usrInput == '0':

            # Position 0    Elevate to 0.6m from ground
            cf.high_level_commander.go_to(0, 0, 0.6, 0, flight_time, relative)
            print('Pos 0')
            time.sleep(flight_time)

        elif usrInput == '1':

            # Position 1  First Quad   
            cf.high_level_commander.go_to(range, range, 0.6, 0, flight_time, relative)
            print('Pos 1')
            time.sleep(flight_time)

        elif usrInput == '2':

            # Position 2   Second Quad
            cf.high_level_commander.go_to(-range, range, 0.6, 0, flight_time, relative)
            print('Pos 2')
            time.sleep(flight_time)

        elif usrInput == '3':

            # Position 3   Third Quad
            cf.high_level_commander.go_to(-range, -range, 0.6, 0, flight_time, relative)
            print('Pos 3')
            time.sleep(flight_time)

        elif usrInput == '4':

            # Position 4    Fourth Quad
            cf.high_level_commander.go_to(range, -range, 0.6, 0, flight_time, relative)
            print('Pos 4')
            time.sleep(flight_time)

        elif usrInput == '5':

            # Position 5    Lower for Landing 
            cf.high_level_commander.go_to(0, 0, 0.2, 0, flight_time, relative)
            print('Pos 5')
            time.sleep(flight_time)

        if usrInput == '.':

            # Exiting Function
            cf.high_level_commander.land(0.0, 1)
            time.sleep(1)
            print('Exiting User Input... ')
            break

# =======================================================================================================================================================




# The Following Section Contained the codes for the drone to run a trajectory of an 8 Figure ============================================================
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D

# The trajectory to fly
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    upload_result = trajectory_mem.write_data_sync()
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration


def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(0.8, 3.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.go_to(0, 0, 0.3, 0, 5, relative)
    time.sleep(2)
    commander.land(0.0, 3.0)
    time.sleep(2)
    commander.stop()

# =======================================================================================================================================================




# The Main Function of the MocapSingleDrone.py ==========================================================================================================
if __name__ == '__main__':
    # Instantiate the Crazyflie Driver
    cflib.crtp.init_drivers()
    
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

            # Create a single drone object
            drone = scf.cf

            # Connect to OptiTrack 
            crazymocap_warpper = CrazyMoCap(drone)    
           
            # start_position_printing(drone)
            # start_orientation_printing(drone)
            # start_motor_printing(drone)

            print('\n============= SETTING UP DRONES =============')
            adjust_orientation_sensitivity(drone)
            activate_kalman_estimator(drone)
            activate_high_level_commander(drone)
            # activate_mellinger_controller(drone)        # Working version DO NOT have this enabled! (use PID)
            drone_ready_flag = True
            reset_estimator(drone)                   # Need!
            print('=============================================\n')

            user_input(drone, 0.6, 3)
            
            # duration = upload_trajectory(drone, 1, figure8)
            # print('The sequence is {:.1f} seconds long'.format(duration*2))
            # run_sequence(drone, 1, duration*2)

        crazymocap_warpper.close()
        scf.close_link()
        print('Sequence Done!  Closing Link...')

    except KeyboardInterrupt:
        print("Keyboard Interruptted!  Closing Link.... ")

        drone.high_level_commander.land(0.00, 0.5)
        print('Emergency Landing!! ')
        scf.close_link()
    

