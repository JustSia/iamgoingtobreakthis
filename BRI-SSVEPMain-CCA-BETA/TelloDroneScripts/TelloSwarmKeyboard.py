#### pip install tello-edu-py

from pynput import keyboard
from djitellopy import Tello
from djitellopy import TelloSwarm

rangee = 80
velocity = 100
h = 50

droneMiddle = [  # Crazy-30 (Light)
     (0, 0, h, velocity),      # Highest 7
     (0, 0, h, velocity),      # Lowest  8
     (-rangee,  rangee, h, velocity),        # Pos1
     (-rangee, 0      , h, velocity),        # Pos2
     (-rangee, -rangee, h, velocity),        # Pos3
     ( rangee,  rangee, h+h, velocity),      # Pos4
     ( rangee, 0      , h+h, velocity),      # Pos5
     ( rangee, -rangee, h+h, velocity),      # Pos6

]

class KeyboardDrone:

    def __init__(self, tello):
        
        self.distance = 50
        self.ang_velocity = 45
        self.tello = tello

        tello.connect()
        tello.set_speed(100)
        tello.streamon()
        print('Press t for taking off!')

    def on_press(self, key):

        if key.char == 'l':
            self.tello.land()
        if key.char == 't':
            self.tello.takeoff()   
        if key.char == 'w':
            self.tello.move_forward(self.distance)
        if key.char == 's':
            self.tello.move_back(self.distance)
        if key.char == 'a':
            self.tello.move_left(self.distance)
        if key.char == 'd':
            self.tello.move_right(self.distance)
        if key.char == 'e':
            self.tello.rotate_clockwise(self.ang_velocity )
        if key.char == 'q':
            self.tello.rotate_counter_clockwise(self.ang_velocity )
        if key.char == '.':
            self.tello.move_up(self.distance)
        if key.char == ',':
            self.tello.move_down(self.distance)
        if key.char == 'u':
            self.tello.flip_forward()
        if key.char == 'j':
            self.tello.flip_back()
        if key.char == 'h':
            self.tello.flip_left()
        if key.char == 'k':
            self.tello.flip_right()
        if key.char == '1':
            self.tello.go_xyz_speed_mid(droneMiddle[2][0], droneMiddle[2][1], droneMiddle[2][2], droneMiddle[2][3],1)
        if key.char == '2':
            self.tello.go_xyz_speed_mid(droneMiddle[3][0], droneMiddle[3][1], droneMiddle[3][2], droneMiddle[3][3],1)
        if key.char == '3':
            self.tello.go_xyz_speed_mid(droneMiddle[4][0], droneMiddle[4][1], droneMiddle[4][2], droneMiddle[4][3],1)
        if key.char == '0':
            self.tello.go_xyz_speed_mid(droneMiddle[1][0], droneMiddle[1][1], droneMiddle[1][2], droneMiddle[1][3],1)



if __name__ == '__main__':

    telloswarm = TelloSwarm.fromIps(["192.168.0.220"])   #220

    # telloswarm.connect_to_wifi("BRIEEG32","BRIEEG32")    # ---> working use 2.4GHz wifi only
    
    drone = KeyboardDrone(telloswarm)

    with keyboard.Listener(on_press=drone.on_press) as listener:
        listener.join()


