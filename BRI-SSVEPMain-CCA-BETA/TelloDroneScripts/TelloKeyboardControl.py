#### pip install tello-edu-py

# only Suitable for a single drone. If you connect your pc to the drone's wifi.

from pynput import keyboard
from djitellopy import Tello

class KeyboardDrone:

    def __init__(self, tello):
        
        self.distance = 100
        self.ang_velocity = 45
        self.tello = tello

        tello.connect()

        tello.set_speed(100)
        
        print('Press t for taking off!') 
        print('Press x for throwing take off :D')

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
            self.tello.go_xyz_speed(80,80,80,100)
        if key.char == '2':
            self.tello.go_xyz_speed(80,0,80,100)
        if key.char == '3':
            self.tello.go_xyz_speed(80,-80,80,100)
        if key.char == '4':
            self.tello.go_xyz_speed(-80,80,100,100)
        if key.char == '5':
            self.tello.go_xyz_speed(-80,0,100,100)
        if key.char == '6':
            self.tello.go_xyz_speed(-80,-80,100,100)
        if key.char == '0':
            self.tello.go_xyz_speed(0,0,80,100)
        if key.char == '7':
            self.tello.go_xyz_speed(0,0,40,100)
        if key.char == 'x':
            self.tello.initiate_throw_takeoff()

if __name__ == '__main__':

    tello = Tello()

    # tello.connect_to_wifi("BRI_Router_5.2GHz","cibci000")
    

    drone = KeyboardDrone(tello)

    with keyboard.Listener(on_press=drone.on_press) as listener:
        listener.join()




# from djitellopy import TelloSwarm
# import pygame

# # initialize pygame
# pygame.init()
# screen = pygame.display.set_mode((800, 600))
# pygame.display.set_caption("Tello Swarm Control")

# # create a swarm of 4 drones
# swarm = TelloSwarm.fromIps(["192.168.10.1"])

# # connect to the swarm
# swarm.connect()
# swarm.streamon()

# # define some constants for the keyboard commands
# UP = pygame.K_UP
# DOWN = pygame.K_DOWN
# LEFT = pygame.K_LEFT
# RIGHT = pygame.K_RIGHT
# W = pygame.K_w
# S = pygame.K_s
# A = pygame.K_a
# D = pygame.K_d
# T = pygame.K_t
# L = pygame.K_l

# # define the speed and the increment
# SPEED = 50
# INCREMENT = 10

# # start the main loop
# running = True
# while running:
#     # handle the events
#     for event in pygame.event.get():
#         # quit the program if the user closes the window or presses ESC
#         if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
#             running = False
        
#         # handle the key presses
#         elif event.type == pygame.KEYDOWN:
#             # take off or land the swarm with T or L keys
#             if event.key == T:
#                 swarm.takeoff()
#             elif event.key == L:
#                 swarm.land()
            
#             # increase or decrease the speed with UP or DOWN keys
#             elif event.key == UP:
#                 SPEED += INCREMENT
#                 if SPEED > 100:
#                     SPEED = 100
#                 print(f"Speed: {SPEED}")
#             elif event.key == DOWN:
#                 SPEED -= INCREMENT
#                 if SPEED < 0:
#                     SPEED = 0
#                 print(f"Speed: {SPEED}")
            
#             # move the swarm with W, A, S, D keys
#             elif event.key == W:
#                 swarm.move_forward(SPEED)
#             elif event.key == S:
#                 swarm.move_back(SPEED)
#             elif event.key == A:
#                 swarm.move_left(SPEED)
#             elif event.key == D:
#                 swarm.move_right(SPEED)
            
#             # rotate the swarm with LEFT or RIGHT keys
#             elif event.key == LEFT:
#                 swarm.rotate_counter_clockwise(SPEED)
#             elif event.key == RIGHT:
#                 swarm.rotate_clockwise(SPEED)
    
#     # update the screen with the video stream from the first drone in the swarm
#     screen.fill((0, 0, 0))
#     frame = swarm.get_frame_read(0).frame # get the frame from the first drone (index 0)
#     if frame is not None:
#         frame = pygame.surfarray.make_surface(frame)
#         screen.blit(frame, (0, 0))
    
#     # update the display
#     pygame.display.flip()

# # end the program and disconnect from the swarm
# pygame.quit()
# swarm.end()
