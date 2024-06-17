# Code for Lighthouse-based drone swarm with HoloLens BCI Control  --------------------------------------------
# 16/Mar/2023 
# Swarm User Input using LightHouse Working!!!    # Three Drones!! 
# Tested pc input 16/03/2023
# Tested hololens input 23/03/2023
# All drones facing motion platform
# Waypoints were all set to be at the same height, one point at each quadrant 

"""
The layout of the positions:                              LH is Lighthouse

LH2        Computers        LH1

       -x            x
                
               ^ Y           
 y             |
       2nd    40      1st  
               |
              30-------> X           All drones facing motion platform
                
       3rd    80      4th
-y

LH3           Door          LH4

"""

"""  
Waypoint Poses

        2             1
                              

              5/6                   5 Middle high      6 Middle low       0 for landing

                              
        3             4     

"""


import time

# --- BCI --- 
import socket
import struct
# -----------

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.swarm import Swarm

rangee = 0.75
flight_time = 2
relative = False
relativee = True

h = 0.5  # remain constant height similar to take off height

xL, yL = 0.0, +0.4   # Left
xR, yR = 0.0, -0.4   # Right


uris = [
    'radio://0/40/2M/E7E7E7E7E7',  # cf_id 0, startup position [ 0, +0.4 ]     (Left)
    'radio://0/10/2M/E7E7E7E7E7',  # cf_id 1, startup position [ 0,  0.0 ]     (with light)
    'radio://0/80/2M/E7E7E7E7E7',  # cf_id 2, startup position [ 0, -0.4 ]     (Right)
    # Add more URIs if you want more copters in the swarm 
]


droneLeft = [  # Crazy-10
     (xL, yL, h+0.4, flight_time),      # Highest
     (xL, yL, h-0.2, flight_time),      # Lowest
     (xL+rangee, yL+rangee, h, flight_time),      # Pos1
     (xL-rangee, yL+rangee, h, flight_time),      # Pos2
     (xL-rangee, yL-rangee, h, flight_time),      # Pos3
     (xL+rangee, yL-rangee, h, flight_time),      # Pos4

]

droneMiddle = [  # Crazy-30 (Light)
     (0, 0, h+0.4, flight_time),      # Highest
     (0, 0, h-0.2, flight_time),      # Lowest
     ( rangee,  rangee, h, flight_time),      # Pos1
     (-rangee,  rangee, h, flight_time),      # Pos2
     (-rangee, -rangee, h, flight_time),      # Pos3
     ( rangee, -rangee, h, flight_time),      # Pos4

]

droneRight = [   # Crazy-80
     (xR, yR, h+0.4, flight_time),      # Highest
     (xR, yR, h-0.2, flight_time),      # Lowest
     (xR+rangee, yR+rangee, h, flight_time),      # Pos1
     (xR-rangee, yR+rangee, h, flight_time),      # Pos2
     (xR-rangee, yR-rangee, h, flight_time),      # Pos3
     (xR+rangee, yR-rangee, h, flight_time),      # Pos4

]


seq_args = {
    uris[0]: [droneLeft],
    uris[1]: [droneMiddle],
    uris[2]: [droneRight],
    # uris[3]: [sequence3],
}



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



def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(scf, use_mellinger):
    controller = 1
    if use_mellinger:
        controller = 2
    scf.cf.param.set_value('stabilizer.controller', controller)


def PosHigh(scf: SyncCrazyflie, Poses): 
    # Position High   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[0][0], Poses[0][1], Poses[0][2]
    duration = Poses[0][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Highest Middle')
    time.sleep(flight_time)


def PosLow(scf: SyncCrazyflie, Poses): 
    # Position Low    
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[1][0], Poses[1][1], Poses[1][2]
    duration = Poses[1][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Lowest Middle')
    time.sleep(flight_time)


def Pos1(scf: SyncCrazyflie, Poses): 
    # Position 1  First Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[2][0], Poses[2][1], Poses[2][2]
    duration = Poses[2][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 1')
    time.sleep(flight_time)


def Pos2(scf: SyncCrazyflie, Poses): 
    # Position 2  Second Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[3][0], Poses[3][1], Poses[3][2]
    duration = Poses[3][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 2')
    time.sleep(flight_time)


def Pos3(scf: SyncCrazyflie, Poses): 
    # Position 3  Third Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[4][0], Poses[4][1], Poses[4][2]
    duration = Poses[4][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 3')
    time.sleep(flight_time)


def Pos4(scf: SyncCrazyflie, Poses): 
    # Position 4  Fourth Quad   
    cf = scf.cf
    commander = scf.cf.high_level_commander

    x, y, z = Poses[5][0], Poses[5][1], Poses[5][2]
    duration = Poses[5][3]

    print('Setting position {} to cf {}'.format((x, y, z), cf.link_uri))
    commander.go_to(x, y, z, 0, duration, relative)
    print('Pos 4')
    time.sleep(flight_time)


def take_off(scf): 
    commander = scf.cf.high_level_commander
    commander.takeoff(0.5, flight_time)
    time.sleep(2)

def landing(scf): 
    commander = scf.cf.high_level_commander
    commander.land(0.0, flight_time)
    time.sleep(2)   

def higher(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(0.0, 0.0, rangee, 0, flight_time, relativee)   # Forward 0.2m
    print('Higher')
    time.sleep(2)     

def lower(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(0.0, 0.0, -rangee, 0, flight_time, relativee)   # Forward 0.2m
    print('Lower')
    time.sleep(2)     

def left(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(0.0, rangee, 0.0, 0, flight_time, relativee)   # Forward 0.2m
    print('Left')
    time.sleep(2) 

def right(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(0.0, -rangee, 0.0, 0, flight_time, relativee)   # Forward 0.2m
    print('Right')
    time.sleep(2)  

def forward(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(rangee, 0.0, 0.0, 0, flight_time, relativee)   # Forward 0.2m
    print('forward')
    time.sleep(2)  

def backward(scf):
    commander = scf.cf.high_level_commander
    commander.go_to(-rangee, 0.0, 0.0, 0, flight_time, relativee)   # Forward 0.2m
    print('backward')
    time.sleep(2)    



if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:

        swarm.parallel_safe(activate_high_level_commander)
        print(swarm.get_estimated_positions())
        swarm.reset_estimators()

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
        print('my IP: ' + str(my_ip))
        print('========================================')

        print('Taking Off!')

        swarm.parallel_safe(take_off)

        print('----- Start Listening Commands -----')

        while True:

            BRI_command = receive_command(sock)
            # BRI_command = input('User Command: ')

            if BRI_command == 1 :
                swarm.parallel_safe(Pos1, args_dict=seq_args)
                print('1')
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            elif BRI_command == 2 :
                swarm.parallel_safe(Pos2, args_dict=seq_args)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                print('2')
            elif BRI_command == 3 :
                swarm.parallel_safe(Pos3, args_dict=seq_args)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                print('3')
            elif BRI_command == 4 :
                swarm.parallel_safe(Pos4, args_dict=seq_args)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                print('4')
            elif BRI_command == 5 :
                swarm.parallel_safe(PosHigh, args_dict=seq_args)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                print('5')
            elif BRI_command == 6 :
                swarm.parallel_safe(PosLow, args_dict=seq_args)
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
                print('6')

            elif BRI_command == 0 :
                swarm.parallel_safe(landing)
                print('0')
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)

            elif BRI_command == -1 :
                swarm.parallel_safe(landing)
                swarm.close_links()
                print('end of program.. ')
                break
