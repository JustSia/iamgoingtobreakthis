# A keyboard controlling script for a single drone ( No BCI )
# To control the drone, press w a s d like in video game.
# TESTED - up and down no working

import logging
from pynput import keyboard

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# URI = 'radio://0/80/2M'  # ENSURE THIS MATCHES YOUR CRAZYFLIE CONFIGURATION
URI = uri_helper.uri_from_env(default='radio://0/50/2M/E7E7E7E7E1')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class KeyboardDrone:

    def __init__(self, mc):
        self.mc = mc

        self.velocity = 0.75
        self.ang_velocity = 120

        self.sleeptime = 0.5
        # self.max_hight = 0.8
        # self.hight = 0.0
        print('Press t for taking off!')

    def on_press(self, key):

        if key.char == 'w':
            self.mc.start_forward(self.velocity)
            print('forward')

        if key.char == 't':
            self.mc.take_off(0.3)
            print('Take off!')

        if key.char == 'l':
            self.mc.land()
            print('Landing')

        if key.char == 's':
            self.mc.start_back(self.velocity)
            print('backward')

        if key.char == 'a':
            self.mc.start_left(self.velocity)
            print('left')

        if key.char == 'd':
            self.mc.start_right(self.velocity)
            print('right')

        if key.char == ',':
            self.mc.start_down(self.velocity)
            print('decending')

        if key.char == '.':
            self.mc.start_up(self.velocity)
            print('ascending')

        if key.char == 'k':
            print('Kill Engines')
            return False

        if key.char == 'q':
            self.mc.start_turn_left(self.ang_velocity)
            print('Rotate Left')

        if key.char == 'e':
            self.mc.start_turn_right(self.ang_velocity)
            print('Rotate Right')

    def on_release(self, key):
        self.mc.stop()


if __name__ == '__main__':

    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        # We take off when the commander is created
        mc = MotionCommander(scf)

        drone = KeyboardDrone(mc)

        with keyboard.Listener(on_press=drone.on_press, on_release=drone.on_release) as listener:
            listener.join()