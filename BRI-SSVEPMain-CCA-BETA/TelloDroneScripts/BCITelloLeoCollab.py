# Code for Tello drone (single) + Leo with HoloLens BCI Control --------------------------------------------
# 14/Set/2023 
# Working with the BRI script!!!! 
# Must use Mission Pad one, rocket facing kitchen.

"""  
Waypoint Poses

        1             4
                              

        2     7/8     5             7 Middle high      8 Middle low       9 for landing

                              
        3             6     

"""

from djitellopy import TelloSwarm
import time

# --- BCI --- 
import socket
import struct
# -----------

rangee = 0.8 # range for crazyflie
h = 0.5  # remain constant height similar to take off height
xL, yL = 0.0, +0.4   # Left crazyflie
xR, yR = 0.0, -0.4  # Right crazyflie

heightT = int(h * 100)
rangeT = int(rangee * 100)
velocity = 100

sendFeedback = True

telloMiddle = [  # Crazy-30 (Light)
     (0, 0, heightT, velocity),      # Above Leo 7
     (0, 0, heightT, velocity),      # Centre    8
     (-rangeT,  rangeT, heightT*2, velocity),      # Pos1
     ( rangeT, 0      , heightT*2, velocity),      # Pos5
     (-rangeT, -rangeT, heightT*2, velocity),      # Pos3
     ( rangeT,  rangeT, heightT*2, velocity),      # Pos4 (Not Used)
     ( rangeT, 0      , heightT*2, velocity),      # Pos5 (Not Used)
     ( rangeT, -rangeT, heightT*2, velocity),      # Pos6 (Not Used)
]


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


# ===========================================================================

if __name__ == '__main__':

    print('========== SETTING UP Drone Swarm ==============')
    telloswarm = TelloSwarm.fromIps(["192.168.0.220"])  #219
    telloswarm.connect()
    telloswarm.set_speed(velocity)
    telloswarm.streamon()
    print('Tello Swarm is Ready!! ')

    print('========== SETTING UP BCI ==============')
    my_ip=socket.gethostbyname(socket.gethostname())
    # initailize receiving socket
    UDP_PORT = 5007 #5005
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

    # print('TAKING OFF!! ')
    # telloswarm.takeoff()
    #telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
    once = 0 
    currentPos = 0
    isFlying = False
    print('All Ready To Go!')
    print('-1 - Kill Program')
    print(' 0 - Return to Middle')
    print(' 4 - Unlock Drone Takeoff')
    print(' 7 - Leo Landing Pose')
    print(' 9 - Landing !!')
    print('10 - Take off')
    print('----- Start Listening Commands -----')
    inpos = False

    while True:
        BRI_command = receive_command(sock)
        # BRI_command = input('User Command: ')
        
        if (BRI_command != None):
            if (BRI_command <= 3): # and BRI_command != 0):
                currentPos = BRI_command
            if (BRI_command == 7):   # Above Leo for Landing
                currentPos = -1
            
        if BRI_command == 1 and inpos :
            telloswarm.takeoff()
            isFlying = True
            telloswarm.go_xyz_speed_mid(telloMiddle[2][0], telloMiddle[2][1], telloMiddle[2][2], telloMiddle[2][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('1')

        elif BRI_command == 2 and inpos :
            
            telloswarm.go_xyz_speed_mid(telloMiddle[3][0], telloMiddle[6][1], telloMiddle[6][2], telloMiddle[6][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('2 - Go to Kitchen')

        elif BRI_command == 3 and inpos :
            
            telloswarm.go_xyz_speed_mid(telloMiddle[4][0], telloMiddle[4][1], telloMiddle[4][2], telloMiddle[4][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('3')

        elif BRI_command == 4 :
            inpos = True
        #     # telloswarm.go_xyz_speed_mid(telloMiddle[5][0], telloMiddle[5][1], telloMiddle[5][2], telloMiddle[5][3],1)
        #     if sendFeedback:
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #     print('4')

        # elif BRI_command == 5 :
        #     telloswarm.go_xyz_speed_mid(telloMiddle[6][0], telloMiddle[6][1], telloMiddle[6][2], telloMiddle[6][3],1)
        #     if sendFeedback:
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #     print('5')

        # elif BRI_command == 6 :     
        #     telloswarm.go_xyz_speed_mid(telloMiddle[7][0], telloMiddle[7][1], telloMiddle[7][2], telloMiddle[7][3],1)
        #     if sendFeedback:
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #     print('6')

        elif BRI_command == 7 :     # Above Leo Rover 
            telloswarm.go_xyz_speed_mid(telloMiddle[0][0], telloMiddle[0][1], telloMiddle[0][2], telloMiddle[0][3], 5)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('7')

        # elif BRI_command == 8 :     # Middle Low Position (prepare for landing)
        #     telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
        #     if sendFeedback:
        #         send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
        #     print('8') 

        elif BRI_command == 0 :     # When BCI got an error input, return to the middle 
            telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('Wrong Signal - Back to Origin')     # Maybe change it to do NOTHING

        elif BRI_command == 9 :
            telloswarm.land()
            print('LANDING')
            # if sendFeedback:
            #     send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)  
        elif BRI_command == 10 :
            telloswarm.takeoff()
            print('Takeoff')  
        
        if BRI_command == None and isFlying == True:
            time.sleep(1)
            telloswarm.go_xyz_speed_mid(telloMiddle[currentPos+1][0], telloMiddle[currentPos+1][1], telloMiddle[currentPos+1][2], telloMiddle[currentPos+1][3],1)
        

        elif BRI_command == -1 :
            telloswarm.land()
            telloswarm.end()
            print('End of Program... ')
            break
