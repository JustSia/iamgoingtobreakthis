# Code by Charlie Tsai
# 25/12/2023
# Fully Working Single Drone Keyboard Control Script with Obstacle Avoidance.

# Crazyflie 2.1 (Single)
    # 1. MultiRanger Deck 
    # 2. Optical Flow Deck v2

import sys
import time
import logging
from pynput import keyboard
from threading import Thread

import cflib.crtp
from cflib.utils import uri_helper
from cflib.crazyflie import Crazyflie
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = uri_helper.uri_from_env(default='radio://0/120/2M/E7E7E7E7E7')

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
velocity = 0.5
ang_velocity = 120

# --- Keyboard --------------------------------------------------------------
class KeyboardDrone:

    def __init__(self, mc):
        self.mc = mc
        self.ctrl_pressed = False
        self.flying = False
        print('Press t to Take Off!')


    def on_press(self, key):    # Minor Bug: Ctrl+C sometimes can't shut down thread fully, and other error msg.
        global currentKey

        if hasattr(key, 'char'):

            if key.char == 't':
                currentKey = 't'
                self.flying = True
                print('Take off!')

            if key.char == 'l':
                currentKey = 'l'
                self.flying = False
                print('Landing')

            if key.char == 'w' and self.flying == True:
                currentKey = 'w'
                print('forward')

            if key.char == 's' and self.flying == True:
                currentKey = 's'
                print('backward')

            if key.char == 'a' and self.flying == True:
                currentKey = 'a'
                print('left')

            if key.char == 'd' and self.flying == True:
                currentKey = 'd'
                print('right')

            if key.char == ',' and self.flying == True:
                currentKey = ','
                print('decending')

            if key.char == '.' and self.flying == True:
                currentKey = '.'
                print('ascending')

            if key.char == 'k' and self.flying == True:
                print('Kill Engines')
                return False

            if key.char == 'q' and self.flying == True:
                currentKey = 'q'
                print('Rotate Left')

            if key.char == 'e' and self.flying == True:
                currentKey = 'e'
                print('Rotate Right')


    def on_release(self, key):
        global currentKey
        currentKey = 'n'
        self.ctrl_pressed = False

# ---------------------------------------------------------------------------

# == keyboardInput Thread Function ===========================================================
def keyboardInput(key):

    while not exit_flag_keyboard:
        with keyboard.Listener(on_press=key.on_press, on_release=key.on_release) as listener:
            listener.join()

    print('Keyboard Thread Ended! ')
    listener.join()


def keyToMotionCommander(mc, key):
    global isFlying
    global senseObstacle

    if key == 't':
        isFlying = True
        mc.take_off(0.3)

    if key == 'l':
        isFlying = False
        mc.land()

    if key == 'k':
        isFlying = False

    if key == 'q':
        mc.start_turn_left(ang_velocity)

    if key == 'e':
        mc.start_turn_right(ang_velocity)
    
    if key == 'n' and isFlying:
        mc.stop()


    if senseObstacle == False:
        if key == 'w':
            mc.start_forward(velocity)

        if key == 's':
            mc.start_back(velocity)

        if key == 'a':
            mc.start_left(velocity)

        if key == 'd':
            mc.start_right(velocity)

        if key == ',':
            mc.start_down(velocity)

        if key == '.':
            mc.start_up(velocity)

# ============================================================================================



# == MultiRanger Thread Function =============================================================
def is_close(range):
    global tooClose
    OBS_DISTANCE = 0.5  # m
    MIN_DISTANCE = 0.2  # 20 cm

    if range is None:
        tooClose = False
        return False
    else:
        if range <= MIN_DISTANCE:
            tooClose = True
        else:
            tooClose = False
        return range < OBS_DISTANCE
    
    
def avoid_obstacle(motion_commander):

    global multirangerReading
    global avoidVelocity
    global senseObstacle

    while not exit_flag_multiranger:
        VELOCITY = 0.5
        velocity_x = 0.0
        velocity_y = 0.0
        keep_flying = 1.0
        senseObstacle = False

        multirangerReading = [multiranger.front, multiranger.back, multiranger.left, multiranger.right, multiranger.up, multiranger.down]

        if is_close(multiranger.front):
            velocity_x -= VELOCITY
            senseObstacle = True
        if is_close(multiranger.back):
            velocity_x += VELOCITY
            senseObstacle = True

        if is_close(multiranger.left):
            velocity_y -= VELOCITY
            senseObstacle = True
        if is_close(multiranger.right):
            velocity_y += VELOCITY
            senseObstacle = True

        if is_close(multiranger.up):
            keep_flying = 0.0
            senseObstacle = True

        avoidVelocity = [velocity_x, velocity_y, keep_flying]
        #print(avoidVelocity[2])
        # Update the sensor values on the same line with error handling
        print('\rFront: {:.2f} | Back: {:.2f} | Left: {:.2f} | Right: {:.2f} | Up: {:.2f} | Down: {:.2f} | Fly: {:.2f}   '.format(
            multiranger.front if multiranger.front is not None else 0.0,
            multiranger.back if multiranger.back is not None else 0.0,
            multiranger.left if multiranger.left is not None else 0.0,
            multiranger.right if multiranger.right is not None else 0.0,
            multiranger.up if multiranger.up is not None else 0.0,
            multiranger.down if multiranger.down is not None else 0.0,
            avoidVelocity[2] if avoidVelocity[2] is not None else 0.0), end='')
        
    print('MultiRanger Thread Ended! ')

# ============================================================================================
    


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        try:
            with Multiranger(scf) as multiranger:
                motion_commander = MotionCommander(scf)
                #motion_commander.take_off()

                global isFlying
                isFlying = False
                keep_flying = True

                global multirangerReading
                multirangerReading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # [Front, Back, Left, Right, Up, Down]

                global avoidVelocity
                avoidVelocity = [0.0, 0.0, 0.0]
                # [forward/backward, left/right, still flying?]

            # -- Multiranger Thread -------------------------------------------------------------
                global senseObstacle
                global tooClose
                exit_flag_multiranger = False
                multirangerThread = Thread(target=avoid_obstacle, args=(motion_commander,))
                multirangerThread.start()
            # -----------------------------------------------------------------------------------

            # -- Keyboard Thread ----------------------------------------------------------------
                exit_flag_keyboard = False
                global currentKey
                currentKey = 'n'
                key = KeyboardDrone(motion_commander)
                keyboardThread = Thread(target=keyboardInput, args=(key,))
                keyboardThread.start()
            # -----------------------------------------------------------------------------------    
                  
                while keep_flying:
                    keep_flying = bool(avoidVelocity[2])

                    keyToMotionCommander(motion_commander, currentKey)

                    if isFlying == True:

                        if senseObstacle == True:
                            motion_commander.start_linear_motion(
                                avoidVelocity[0], avoidVelocity[1], 0)
                            print('Obstacle Inbound. Reversing..')

                            if tooClose == True:
                                motion_commander.stop()
                                print('TOO CLOSE!! STOPPING! ')

                      
                    time.sleep(0.1)
                print('Exit Still flying loop.')

                if avoidVelocity[2] == 0:
                    motion_commander.land()      # Remember to add landing, or else the script won't end. Thread cannot join.


            print('Script Terminated!')
            exit_flag_multiranger = True
            exit_flag_keyboard = True
            multirangerThread.join()
            keyboardThread.join()
            scf.close_link()


        except KeyboardInterrupt:
            print("Keyboard Interruptted!  Closing Link.... ")
            scf.cf.high_level_commander.land(0.01, 0.1)
            exit_flag_multiranger = True
            exit_flag_keyboard = True
            multirangerThread.join()
            keyboardThread.join()
            scf.close_link()



# =============== BACK UP ====================================================
            
#             import sys
# import time
# import logging
# from pynput import keyboard
# from threading import Thread

# import cflib.crtp
# from cflib.utils import uri_helper
# from cflib.crazyflie import Crazyflie
# from cflib.utils.multiranger import Multiranger
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander

# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# if len(sys.argv) > 1:
#     URI = sys.argv[1]

# # Only output errors from the logging framework
# logging.basicConfig(level=logging.ERROR)

# # --- Keyboard --------------------------------------------------------------

# class KeyboardDrone:

#     def __init__(self, mc):
#         self.mc = mc

#         self.velocity = 0.75
#         self.ang_velocity = 120

#         self.sleeptime = 0.5
#         # self.max_hight = 0.8
#         # self.hight = 0.0
#         print('Press t for taking off!')

#     def on_press(self, key):

#         if key.char == 'w':
#             self.mc.start_forward(self.velocity)
#             print('forward')

#         if key.char == 't':
#             self.mc.take_off(0.3)
#             print('Take off!')

#         if key.char == 'l':
#             self.mc.land()
#             print('Landing')

#         if key.char == 's':
#             self.mc.start_back(self.velocity)
#             print('backward')

#         if key.char == 'a':
#             self.mc.start_left(self.velocity)
#             print('left')

#         if key.char == 'd':
#             self.mc.start_right(self.velocity)
#             print('right')

#         if key.char == ',':
#             self.mc.start_down(self.velocity)
#             print('decending')

#         if key.char == '.':
#             self.mc.start_up(self.velocity)
#             print('ascending')

#         if key.char == 'k':
#             print('Kill Engines')
#             return False

#         if key.char == 'q':
#             self.mc.start_turn_left(self.ang_velocity)
#             print('Rotate Left')

#         if key.char == 'e':
#             self.mc.start_turn_right(self.ang_velocity)
#             print('Rotate Right')

#     def on_release(self, key):
#         self.mc.stop()

# # ---------------------------------------------------------------------------

# def is_close(range):
#     MIN_DISTANCE = 0.2  # m

#     if range is None:
#         return False
#     else:
#         return range < MIN_DISTANCE


# def avoid_obstacle(motion_commander):

#     global multirangerReading
#     global avoidVelocity

#     while not exit_flag:
#         VELOCITY = 0.5
#         velocity_x = 0.0
#         velocity_y = 0.0
#         keep_flying = 1.0

#         multirangerReading = [multiranger.front, multiranger.back, multiranger.left, multiranger.right, multiranger.up, multiranger.down]

#         if is_close(multiranger.front):
#             velocity_x -= VELOCITY
#         if is_close(multiranger.back):
#             velocity_x += VELOCITY

#         if is_close(multiranger.left):
#             velocity_y -= VELOCITY
#         if is_close(multiranger.right):
#             velocity_y += VELOCITY

#         if is_close(multiranger.up):
#             keep_flying = 0.0

#         avoidVelocity = [velocity_x, velocity_y, keep_flying]
#         #print(avoidVelocity[2])
#         # Update the sensor values on the same line with error handling
#         print('\rFront: {:.2f} | Back: {:.2f} | Left: {:.2f} | Right: {:.2f} | Up: {:.2f} | Down: {:.2f} | Fly: {:.2f}   '.format(
#             multiranger.front if multiranger.front is not None else 0.0,
#             multiranger.back if multiranger.back is not None else 0.0,
#             multiranger.left if multiranger.left is not None else 0.0,
#             multiranger.right if multiranger.right is not None else 0.0,
#             multiranger.up if multiranger.up is not None else 0.0,
#             multiranger.down if multiranger.down is not None else 0.0,
#             avoidVelocity[2] if avoidVelocity[2] is not None else 0.0), end='')



# if __name__ == '__main__':
#     # Initialize the low-level drivers
#     cflib.crtp.init_drivers()

#     cf = Crazyflie(rw_cache='./cache')
#     with SyncCrazyflie(URI, cf=cf) as scf:
#         try:
#                 motion_commander = MotionCommander(scf)
#             #with MotionCommander(scf) as motion_commander:
#                 with Multiranger(scf) as multiranger:
#                     motion_commander.take_off()
#                     keep_flying = True
#                     global multirangerReading
#                     multirangerReading = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#                     # [Front, Back, Left, Right, Up, Down]

#                     global avoidVelocity
#                     avoidVelocity = [0.0, 0.0, 0.0]
#                 # -----------------------------------------------------------------------------------
#                     exit_flag = False
#                     multirangerThread = Thread(target=avoid_obstacle, args=(motion_commander,))
#                     multirangerThread.start()
#                 # -----------------------------------------------------------------------------------
                    
#                     while keep_flying:

#                         keep_flying = bool(avoidVelocity[2])

#                         motion_commander.start_linear_motion(
#                             avoidVelocity[0], avoidVelocity[1], 0)
                        
#                         time.sleep(0.1)

#                 print('Script Terminated!')


#         except KeyboardInterrupt:
#             print("Keyboard Interruptted!  Closing Link.... ")
#             scf.cf.high_level_commander.land(0.01, 0.1)
#             exit_flag = True
#             multirangerThread.join()
#             scf.close_link()

