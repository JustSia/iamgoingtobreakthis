# SUCCESSFULLY USE BCI and OPTITRACK MOCAP TO FLY CRAZYFLIE!!!!

# 01/March/2023   (First: no full pose)

# BCI Single Drone Only.  (With Full Pose from optiTrack!!!)
# 09/March/2023 !!!!!!!

# Need Stability Test!!!!!!! 

import time
import math
import sys
import socket
import struct

from NatNetClient import NatNetClient

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
# uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')            # Red
uri = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E7E7')  # Blue

# True: send position and orientation;    False: send position only
send_full_pose = True

# True: When all the paramaters are set, and ready fro flying;    False: if otherwise
drone_ready_flag = False

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90 degrees.
# If this is a problem, increase orientation_std_dev a bit.
orientation_std_dev = 0.04  # for mocap 0.04        # The default value in the firmware is 4.5e-3.            8.0e-3


### Start of Class -------------------------------------------------------------------------
class CrazyMoCap(Thread):
    def __init__(self, drone):
        Thread.__init__(self)

        self._stay_open = True
        self.on_pose = None
        self.initial_pose_set = False

        # Setup global drone position variables
        self.position = [0.0, 0.0, 0.0]
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.cf = drone

        self.rotation = [0.0, 0.0, 0.0]
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

        is_looping = True

        # time.sleep(1)

        # if streaming_client.connected() is False:
        #     print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        #     try:
        #         sys.exit(2)
        #     except SystemExit:
        #         print("...")
        #     finally:
        #         print(" ")

    def close(self):
        self._stay_open = False
        self.join()
        print('Thread closed')

    # def run(self):

    # This is a callback function that gets connected to the NatNet client
    # and called once per mocap frame.
    def receiveNewFrame(self, data_dict):
        order_list = ["frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                      "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording",
                      "trackedModelsChanged"]
        dump_args = False
        if dump_args == True:
            out_string = "    "
            for key in data_dict:
                out_string += key + "="
                if key in data_dict:
                    out_string += data_dict[key] + " "
                out_string += "/"
            # print(out_string)

    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receiveRigidBodyFrame(self, new_id, position, rotation):
        pass
        self.position = position
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]

        self.rotation = (rotation[0], rotation[1], rotation[2], rotation[3])
        cal_mocap_rpy = quaternion_to_euler(rotation[1], rotation[2], rotation[3], rotation[0])

        self.on_pose = ([position[0], position[1], position[2], cal_mocap_rpy[2]])  # This line NOT NEEDED

        if self.initial_pose_set == False:
            set_initial_position(self.cf, position[0], position[1], position[2], cal_mocap_rpy[2])  # x,y,z,yaw
            self.initial_pose_set = True

        time.sleep(0.01)

        send_extpose_rot_matrix(self.cf, position[0], position[1], position[2], rotation[0], rotation[1], rotation[2],
                                rotation[3])  # (drone, x, y, z, roll, pitch, yaw)

        # print("x: " + str(position[0]) + "  y: " + str(position[1]) + "  z: " + str(position[2]))
        # print("yaw: " + str(rotation[2]) + "  pitch: " + str(rotation[0]) + "  roll: " + str(rotation[1]))
        # print("DEG: yaw: " + str(math.degrees(rotation[2])) + "  pitch: " + str(math.degrees(rotation[0])) + "  roll: " + str(math.degrees(rotation[1])))

    def my_parse_args(arg_list, args_dict):
        # set up base values
        arg_list_len = len(arg_list)
        if arg_list_len > 1:
            args_dict["serverAddress"] = arg_list[1]
            if arg_list_len > 2:
                args_dict["clientAddress"] = arg_list[2]
            if arg_list_len > 3:
                if len(arg_list[3]):
                    args_dict["use_multicast"] = True
                    if arg_list[3][0].upper() == "U":
                        args_dict["use_multicast"] = False

        return args_dict


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

    return roll_x, pitch_y, yaw_z  # in radians


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
        return np.array([1, 0, 0, 0])
    else:
        return q / norm


def send_extpose_rot_matrix(cf, x, y, z, qx, qy, qz, qw):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
    rotaton matrix. This is going to be forwarded to the Crazyflie's
    position estimator.
    """

    # print('Sending Expos to Drone!! ')

    rounded_x = x  # round(x, 5)              # -round(y, 5)
    rounded_y = y  # round(y, 5)              #  round(x, 5)
    rounded_z = z - 0.05  # round(z-0.02, 5)

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


# Motor value callback ---------------------------------------------------------------
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


# Position value callback ---------------------------------------------------------------
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


# orientation value callback ---------------------------------------------------------------
def orientation_callback(timestamp, data, logconf):
    qw = data['kalman.q0']  # qw
    qx = data['kalman.q1']  # qx
    qy = data['kalman.q2']  # qy
    qz = data['kalman.q3']  # qz

    eular = quaternion_to_euler(qw, qx, qy, qz)

    print('Drone Orient DEG: ( Roll: {}, Pitch: {}, Yaw: {})'.format(math.degrees(eular[0]), math.degrees(eular[1]),
                                                                     math.degrees(eular[2])))
    # print('Drone Orient QUAT: ({}, {}, {}, {})'.format(qx, qy, qz, qw))


def start_orientation_printing(drone):
    log_conf = LogConfig(name='Orientation', period_in_ms=50)
    log_conf.add_variable('kalman.q0', 'float')  # qw
    log_conf.add_variable('kalman.q1', 'float')  # qx
    log_conf.add_variable('kalman.q2', 'float')  # qy
    log_conf.add_variable('kalman.q3', 'float')  # qz

    drone.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(orientation_callback)
    log_conf.start()


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)
    print('Resetting Position Estimator..')


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)
    print('Adjusting orientation Sensitivity..')


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    print('Activate Kalman Estimator..')


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')
    print('Activate High Level Commander..')


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')  # 1 for PID and 2 for Mellinger
    print('Activate Mellinger Controller..')


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
    

def user_input(cf, range, flight_time):  # If Low Level control, use PID, If high level control, use mellinger.

    print('----- Running Sequence -----')

    # activate_mellinger_controller(scf)
    box_size = 0.5
    commander = scf.cf.high_level_commander
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
    print('my IP: ' + str(my_ip))

    # initalial take off
    # take_off(commander)
    commander.takeoff(0.3, 3)
    time.sleep(3)

    box_size = range
    relative = False

    print("-----Start Listening Commands-----")

    while True:
        try:
            
            BRI_command = receive_command(sock)

            if BRI_command == 1:
                commander.go_to(0, 0, 0.6, 0, flight_time, relative)
                time.sleep(flight_time)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

            elif BRI_command == 2:
                # Position 2  first quad
                commander.go_to(range, range, 0.6, 0, flight_time, relative)
                time.sleep(flight_time)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

            elif BRI_command == 3:
                # Position 3   second quad
                commander.go_to(-range, range, 0.6, 0, flight_time, relative)
                time.sleep(flight_time)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

            elif BRI_command == 4:
                # Position 4   third quad
                commander.go_to(-range, -range, 0.6, 0, flight_time, relative)
                time.sleep(flight_time)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

            elif BRI_command == 5:
                # Position 5    fourth quad
                commander.go_to(range, -range, 0.6, 0, flight_time, relative)
                time.sleep(flight_time)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

            elif BRI_command == 6:
                # Position 6 Lower for landing
                commander.go_to(0, 0, 0.2, 0, flight_time, relative)
                time.sleep(flight_time)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

            elif BRI_command == 0:
                # Landing 
                commander.land(0.01, 3)
                time.sleep(flight_time)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

            elif BRI_command == -1:
                commander.land(0.01, 2)
                commander.stop()
                print('Emergency Landing!')
                break
            time.sleep(1)

        except KeyboardInterrupt as e:
            print("keyboard Terminate", e)
            break


if __name__ == '__main__':
    # Instantiate the Crazyflie Driver
    cflib.crtp.init_drivers()

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

            drone = scf.cf

            # Connect to OptiTrack
            crazymocap_warpper = CrazyMoCap(drone)

            # start_position_printing(drone)
            # start_orientation_printing(drone)
            # start_motor_printing(drone)

            adjust_orientation_sensitivity(drone)
            activate_kalman_estimator(drone)
            activate_high_level_commander(drone)
            # activate_mellinger_controller(drone)
            drone_ready_flag = True

            reset_estimator(drone)  # Need!
            user_input(drone, 0.5, 2)

        crazymocap_warpper.close()
        scf.close_link()
        print('Sequence Done!  Closing Link...')

    except KeyboardInterrupt:
        print("Exiting and Closing Link.... ")
        # drone.high_level_commander.land(0.00, 0.5)
        scf.close_link()


