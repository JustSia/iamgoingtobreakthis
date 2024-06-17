# Code for Tello drone (single) with HoloLens BCI Control --------------------------------------------
# 18/Aug/2023 
# Must use Mission Pad one, rocket facing kitchen.

"""  
Waypoint Poses

        1             4
                              

        2     7/8     5    ----> rocket       7 Middle high      8 Middle low       9 for landing

                              
        3             6     

"""

from djitellopy import TelloSwarm
import time

# ----------- 
import socket
import struct
# -----------

rangee = 0.8  # Range between different points 
h = 0.5  # remain constant height similar to take off height
xL, yL = 0.0, +0.4   # Left crazyflie
xR, yR = 0.0, -0.4  # Right crazyflie

heightT = int(h * 100)
rangeT = int(rangee * 100)
velocity = 100

telloMiddle = [  # Crazy-30 (Light)
     (0, 0, heightT, velocity),      # Highest 7
     (0, 0, heightT, velocity),      # Lowest  8
     (-rangeT,  rangeT, heightT, velocity),        # Pos1
     (-rangeT, 0      , heightT, velocity),        # Pos2
     (-rangeT, -rangeT, heightT, velocity),        # Pos3
     ( rangeT,  rangeT, heightT*2, velocity),      # Pos4
     ( rangeT, 0      , heightT*2, velocity),      # Pos5
     ( rangeT, -rangeT, heightT*2, velocity),      # Pos6
]


if __name__ == '__main__':

    print('========== SETTING UP Drone Swarm ==============')   # ==== WARNING!!! THIS WILL TAKE OFF ======
    telloswarm = TelloSwarm.fromIps(["192.168.0.220"])          # ==== IMMEDIATELY WHEN RUN ===============
    telloswarm.connect()
    telloswarm.set_speed(velocity)
    telloswarm.streamon()
    print('Tello Swarm is Ready!! ')
    print('========================================')

    print('TAKING OFF!! ')
    currentPos = 0
    telloswarm.takeoff()
    isFlying = True
    telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
    
    print('All Ready To Go!')
    print('----- Start Listening Commands -----')

    while True:

        BRI_command = input('User Command: ')

        if (BRI_command != None):
            if (int(BRI_command) <= 6 and int(BRI_command) != 0):    # 6 or 8 idk 
                currentPos = int(BRI_command)

        if BRI_command == '1' :
            telloswarm.go_xyz_speed_mid(telloMiddle[2][0], telloMiddle[2][1], telloMiddle[2][2], telloMiddle[2][3],1)
            print('1')

        elif BRI_command == '2' :
            telloswarm.go_xyz_speed_mid(telloMiddle[3][0], telloMiddle[3][1], telloMiddle[3][2], telloMiddle[3][3],1)
            print('2')

        elif BRI_command == '3' :
            telloswarm.go_xyz_speed_mid(telloMiddle[4][0], telloMiddle[4][1], telloMiddle[4][2], telloMiddle[4][3],1)
            print('3')

        elif BRI_command == '4' :
            telloswarm.go_xyz_speed_mid(telloMiddle[5][0], telloMiddle[5][1], telloMiddle[5][2], telloMiddle[5][3],1)
            print('4')

        elif BRI_command == '5' :
            telloswarm.go_xyz_speed_mid(telloMiddle[6][0], telloMiddle[6][1], telloMiddle[6][2], telloMiddle[6][3],1)
            print('5')

        elif BRI_command == '6' :     
            telloswarm.go_xyz_speed_mid(telloMiddle[7][0], telloMiddle[7][1], telloMiddle[7][2], telloMiddle[7][3],1)
            print('6')

        elif BRI_command == '7' :     # Middle High Position 
            telloswarm.go_xyz_speed_mid(telloMiddle[0][0], telloMiddle[0][1], telloMiddle[0][2], telloMiddle[0][3],1)
            print('7')

        elif BRI_command == '8' :     # Middle Low Position (prepare for landing)
            telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
            print('8') 

        elif BRI_command == '0' :     # When BCI got an error input, return to the middle 
            telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
            print('Wrong Signal - Back to Origin')

        elif BRI_command == '9' :
            telloswarm.land()
            isFlying = False
            print('LANDING')
   
        elif BRI_command == '10' :
            telloswarm.takeoff()
            isFlying = True
            print('Takeoff')  

        if BRI_command == None and isFlying == True:
            time.sleep(1)
            telloswarm.go_xyz_speed_mid(telloMiddle[currentPos+1][0], telloMiddle[currentPos+1][1], telloMiddle[currentPos+1][2], telloMiddle[currentPos+1][3],1)
        
        elif BRI_command == '-1' :
            telloswarm.land()
            telloswarm.end()
            print('End of Program... ')
            break
