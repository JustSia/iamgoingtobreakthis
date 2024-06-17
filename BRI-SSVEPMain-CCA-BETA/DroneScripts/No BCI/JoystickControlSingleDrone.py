# A keyboard controlling script for a single drone ( No BCI )
# To control the drone, press w a s d like in video game.
# TESTED - up and down no working

import logging
from pynput import keyboard

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

import threading
from inputs import get_gamepad

# URI = 'radio://0/80/2M'  # ENSURE THIS MATCHES YOUR CRAZYFLIE CONFIGURATION
URI = uri_helper.uri_from_env(default='radio://0/69/2M/E7E7E7E7E7')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class XboxControllerInput(threading.Thread):
    def __init__(self):
        super().__init__()
        self.stop_thread = threading.Event()

        # Initialize controller state variables
        self.left_stick_x = 0
        self.left_stick_y = 0
        self.right_stick_x = 0
        self.right_stick_y = 0
        self.buttons = {}

    def run(self):
        while not self.stop_thread.is_set():
            events = get_gamepad()
            for event in events:
                self._process_event(event)

    
    def _process_event(self, event):
        max_degrees = 60000  # Max roll/pitch in degrees
        max_yaw_rate = 60000  # Max yaw rate in degrees/s
        min_thrust = -60000
        max_thrust = 60000

        if event.ev_type == 'Absolute':
            if event.code == 'ABS_X':
                self.left_stick_x = self._scale(event.state, -32768, 32767, -max_degrees, max_degrees)
            elif event.code == 'ABS_Y':
                self.left_stick_y = self._scale(event.state, -32768, 32767, -max_degrees, max_degrees)
            elif event.code == 'ABS_RX':
                self.right_stick_x = self._scale(event.state, -32768, 32767, -max_yaw_rate, max_yaw_rate)
            elif event.code == 'ABS_RY':
                thrust = self._scale(event.state, -32768, 32767, min_thrust, max_thrust)
                self.right_stick_y = max(thrust, 0) if thrust > 0 else 0

    def _scale(self, val, in_min, in_max, out_min, out_max):
        # Scale a value from one range to another
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def stop(self):
        self.stop_thread.set()

    # Add any additional methods you need to retrieve the state of the controller


if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:

        controller = XboxControllerInput()
        controller.start()
        print('Script Ready! ')
        try:
            while True:
                roll = int(controller.right_stick_x)
                pitch = int(controller.right_stick_y)
                yaw = int(controller.left_stick_x)
                thrust = int(controller.left_stick_y)

                if yaw < 500:
                    yaw = 0
                if thrust < 500:
                    thrust = 0

                print('Roll: {}, Pitch: {}, Yaw: {}, Thrust: {}'.format(roll, pitch, yaw, thrust))

                scf.cf.commander.send_setpoint(int(roll), int(pitch), int(yaw), int(thrust))

        except KeyboardInterrupt:
            print("Program terminated by user")
        finally:
            controller.stop()
            controller.join()
            scf.close_link()
            # Make sure to close any connections to the drone and clean up
