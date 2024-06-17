# Code for Tello drone (single) with HoloLens BCI Control --------------------------------------------
# 18/Aug/2023 
# Worling with the BRI script!!!! 
# Must use Mission Pad one, rocket facing kitchen.

# Only use the old main & Z Flow 13: 
# python RunBCI.py --device menta --robhost 192.168.0.202 --trial 24
# Manuel landing only by pressing 9

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
     (0, 0, heightT, velocity),      # Highest 7
     (0, 0, heightT, velocity),      # Lowest  8
     (-rangeT,  rangeT, heightT, velocity),        # Pos1
     (-rangeT, 0      , heightT, velocity),        # Pos2
     (-rangeT, -rangeT, heightT, velocity),        # Pos3
     ( rangeT,  rangeT, heightT*2, velocity),      # Pos4
     ( rangeT, 0      , heightT*2, velocity),      # Pos5
     ( rangeT, -rangeT, heightT*2, velocity),      # Pos6
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
    telloswarm = TelloSwarm.fromIps(["192.168.0.220"])
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

    print('TAKING OFF!! ')
    currentPos = 0
    telloswarm.takeoff()
    isFlying = True
    telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
    
    print('All Ready To Go!')
    print('----- Start Listening Commands -----')


    while True:

        BRI_command = receive_command(sock)
        # BRI_command = input('User Command: ')

        if (BRI_command != None):
            if (BRI_command <= 6 and BRI_command != 0):
                currentPos = BRI_command

        if BRI_command == 1 :
            telloswarm.go_xyz_speed_mid(telloMiddle[2][0], telloMiddle[2][1], telloMiddle[2][2], telloMiddle[2][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('1')

        elif BRI_command == 2 :
            telloswarm.go_xyz_speed_mid(telloMiddle[3][0], telloMiddle[3][1], telloMiddle[3][2], telloMiddle[3][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('2')

        elif BRI_command == 3 :
            telloswarm.go_xyz_speed_mid(telloMiddle[4][0], telloMiddle[4][1], telloMiddle[4][2], telloMiddle[4][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('3')

        elif BRI_command == 4 :
            telloswarm.go_xyz_speed_mid(telloMiddle[5][0], telloMiddle[5][1], telloMiddle[5][2], telloMiddle[5][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('4')

        elif BRI_command == 5 :
            telloswarm.go_xyz_speed_mid(telloMiddle[6][0], telloMiddle[6][1], telloMiddle[6][2], telloMiddle[6][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('5')

        elif BRI_command == 6 :     
            telloswarm.go_xyz_speed_mid(telloMiddle[7][0], telloMiddle[7][1], telloMiddle[7][2], telloMiddle[7][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('6')

        elif BRI_command == 7 :     # Middle High Position 
            telloswarm.go_xyz_speed_mid(telloMiddle[0][0], telloMiddle[0][1], telloMiddle[0][2], telloMiddle[0][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('7')

        elif BRI_command == 8 :     # Middle Low Position (prepare for landing)
            telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('8') 

        elif BRI_command == 0 :     # When BCI got an error input, return to the middle 
            telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)
            print('Wrong Signal - Back to Origin')

        elif BRI_command == 9 :
            telloswarm.land()
            isFlying = False
            print('LANDING')
            if sendFeedback:
                send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT)    
        
        elif BRI_command == 10 :
            telloswarm.takeoff()
            isFlying = True
            print('Takeoff')  

        if BRI_command == None and isFlying == True:
            time.sleep(1)
            telloswarm.go_xyz_speed_mid(telloMiddle[currentPos+1][0], telloMiddle[currentPos+1][1], telloMiddle[currentPos+1][2], telloMiddle[currentPos+1][3],1)
        

        elif BRI_command == -1 :
            telloswarm.land()
            telloswarm.end()
            print('End of Program... ')
            break
