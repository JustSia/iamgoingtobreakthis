# Code  --------------------------------------------
# 16/Mar/2023 
# Swarm User Input using LightHouse Working!!! # Three Drones!! 

import time

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.swarm import Swarm

rangee = 0.5
flight_time = 2
relative = False
relativee = True

h = 0.5  # remain constant height similar to take off height

xL, yL = 0.0, +0.4   # Left
xR, yR = 0.0, -0.4   # Right


uris = [
    'radio://0/40/2M/E7E7E7E7E7',  # cf_id 0, startup position [ 0, +0.4 ]     (Left)
    'radio://0/20/2M/E7E7E7E7E7',  # cf_id 1, startup position [ 0,  0.0 ]     (with light)
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
        print('========================')
        swarm.parallel_safe(take_off)
        while True:
            print('----- Accepting User Keyboard Input -----')
            uinput = input('User Command: ')

            if uinput == '1' :
                swarm.parallel_safe(Pos1, args_dict=seq_args)
            elif uinput == '2' :
                swarm.parallel_safe(Pos2, args_dict=seq_args)
            elif uinput == '3' :
                swarm.parallel_safe(Pos3, args_dict=seq_args)
            elif uinput == '4' :
                swarm.parallel_safe(Pos4, args_dict=seq_args)
            elif uinput == '5' :
                swarm.parallel_safe(PosHigh, args_dict=seq_args)
            elif uinput == '6' :
                swarm.parallel_safe(PosLow, args_dict=seq_args)
            elif uinput == 't' :
                swarm.parallel_safe(take_off)
            elif uinput == 'l' :
                swarm.parallel_safe(landing)

            elif uinput == '.' :
                print('end of program.. ')
                swarm.close_links()
                break