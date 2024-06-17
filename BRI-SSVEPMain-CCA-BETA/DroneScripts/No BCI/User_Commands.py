# Code Below is Written by Charlie Tsai     ( 12/Feb/2023 )
# First Version 
# No OptiTrack Yet (Only Optical Sensors or Base station) --------------------------------------------------------------------------------------------------------

import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
# uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')            # Red
# uri = uri_helper.uri_from_env(default='radio://0/40/2M/E7E7E7E7E7')            # Blue
# uri = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E7')
uri = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E7E7')              # 40 with flow deck

# Change the sequence according to your setup
#             x    y    z


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
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

            print("{} {} {}".
                  format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)



def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('Drone Pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


# def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):                    # NOT USED!!!
#     cf = scf.cf

#     for position in sequence:
#         print('Setting position {}'.format(position))

#         x = position[0] + base_x
#         y = position[1] + base_y
#         z = position[2] + base_z

#         for i in range(50):
#             cf.commander.send_position_setpoint(x, y, z, yaw)
#             time.sleep(0.1)

#     cf.commander.send_stop_setpoint()
#     # Make sure that the last packet leaves before the link is closed
#     # since the message queue is not flushed before closing
#     time.sleep(0.1)


# def set_pose(scf, sequence, base_x, base_y, base_z, yaw):                        # NOT USED!!! 

#     cf = scf.cf

#     for position in sequence:
#         print('Setting position {}'.format(position))

#         x = position[0] + base_x
#         y = position[1] + base_y
#         z = position[2] + base_z

#         for i in range(50):
#             cf.commander.send_setpoint(x, y, z, yaw)
#             time.sleep(0.1)

#     cf.commander.send_stop_setpoint()
#     # Make sure that the last packet leaves before the link is closed
#     # since the message queue is not flushed before closing
#     time.sleep(0.1)

#     cf.commander.send_setpoint(0, 0, 0, 0)

# =========================================================================


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    
    initial_x = 0.0
    initial_y = 0.0
    initial_z = 0.0
    initial_yaw = 0.0  # In degrees             # 0: positive X direction
                                                # 90: positive Y direction
                                                # 180: negative X direction
                                                # 270: negative Y direction

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)

        reset_estimator(scf)            # Only with Optical Sensor
        #start_position_printing(scf)

        cf = scf.cf
        flight_time = 2
        range = 0.4

        # -----------------------------------------------------------------------------------
        cf.param.set_value('locSrv.extQuatStdDev', 8.0e-3)
        cf.param.set_value('stabilizer.estimator', '2')
        cf.param.set_value('commander.enHighLevel', '1')     # Activate high level commander
        cf.param.set_value('stabilizer.controller', '2')

        # -----------------------------------------------------------------------------------

        while True:

            usrInput = input('User Command: ')

            if usrInput == 't':
                # print('t')
                cf.high_level_commander.takeoff(0.3, flight_time)
                time.sleep(2)

            elif usrInput == 'l':
                cf.high_level_commander.land(0.0, flight_time)
                time.sleep(2)    

            elif usrInput == '0':

                # Position 0    elevate to 0.6m from ground
                cf.high_level_commander.go_to(0, 0, 0.6, 0, flight_time, relative=False)
                time.sleep(flight_time)

            elif usrInput == '1':

                # Position 1  first quad
                cf.high_level_commander.go_to(range, range, 0.6, 0, flight_time, relative=False)
                time.sleep(flight_time)

            elif usrInput == '2':

                # Position 2   second quad
                cf.high_level_commander.go_to(-range, range, 0.6, 0, flight_time, relative=False)
                time.sleep(flight_time)

            elif usrInput == '3':

                # Position 3   third quad
                cf.high_level_commander.go_to(-range, -range, 0.6, 0, flight_time, relative=False)
                time.sleep(flight_time)

            elif usrInput == '4':

                # Position 4    fourth quad
                cf.high_level_commander.go_to(range, -range, 0.6, 0, flight_time, relative=False)
                time.sleep(flight_time)

            elif usrInput == '5':

                # Position 5 Lower for landing 
                cf.high_level_commander.go_to(0, 0, 0.2, 0, flight_time, relative=False)
                time.sleep(flight_time)

            if usrInput == '.':
                print('end of program.. ')
                break


