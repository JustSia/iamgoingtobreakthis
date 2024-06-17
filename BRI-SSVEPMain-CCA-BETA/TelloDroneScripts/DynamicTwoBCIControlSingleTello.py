# Code for Tello drone (single) with HoloLens BCI Control --------------------------------------------
# 18/Aug/2023 
# Worling with the BRI script!!!! 
# Must use Mission Pad one, rocket facing kitchen.

# Only use the old main & Z Flow 13: 
# python RunBCI.py --device menta --robhost 192.168.0.202 --trial 24
# Manuel landing only by pressing 9

from djitellopy import TelloSwarm, Tello
import time
import logging
# --- BCI --- 
import socket
import struct
# -----------
import select
import math
import threading
import queue
# ---Video---
import cv2
import socket
import time

# Define ports and addresses
BCI_UDP_PORT = 5007  # Port for BCI-related commands
UNITY_UDP_PORT = 8021  # Port for Unity coordinates
feed_back_PORT = 5006  # Feedback port
HOST_IP = "0.0.0.0"  # Listen on all available interfaces

# Create a UDP socket for port 5007
sock_5007 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_5007.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_5007.bind((HOST_IP, 5007))
sock_5007.setblocking(0)
 
# Create a UDP socket for port 8021
sock_8021 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_8021.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_8021.bind((HOST_IP, 8021))
sock_8021.setblocking(0)

# Feedback socket setup
feed_back_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#feed_back_IP1 = "192.168.0.202"                        # <--------  Control PC IP (The first PC running main)
feed_back_IP1 = "192.168.0.23"   # ROG Ally
feed_back_IP2 = "192.168.0.202"                         # <--------  Control PC IP (The second PC running main)

# Global queues for thread communication
command_queue = queue.Queue()
unity_data_queue = queue.Queue()
quit = False
stop_udp_thread = False
flyingEnabled = True

rangee = 0.8 # range for crazyflie
h = 0.5  # remain constant height similar to take off height
xL, yL = 0.0, +0.4   # Left crazyflie
xR, yR = 0.0, -0.4  # Right crazyflie

heightT = int(h * 100)
rangeT = int(rangee * 100)
velocity = 100

sendFeedback = True

global telloMiddle

telloMiddle = [ 
     (0, 0, heightT, velocity),      # Highest 7
     (0, 0, heightT, velocity),      # Lowest  8
     (-rangeT,  rangeT, heightT, velocity),        # Pos1 BRI_command == 1 
     (-rangeT, 0      , heightT, velocity),        # Pos2 BRI_command == 2
     (-rangeT, -rangeT, heightT, velocity),        # Pos3 BRI_command == 3
     ( rangeT,  rangeT, heightT*2, velocity),      # Pos4 BRI_command == 4 
     ( rangeT, 0      , heightT*2, velocity),      # Pos5 BRI_command == 5
     ( rangeT, -rangeT, heightT*2, velocity),      # Pos6 BRI_command == 6
]

# =================== Video Sreaming (Not Tested Yet)==========================

def video_stream(tello):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('0.0.0.0', 12346))  # Bind to an address and port
    # Start video stream
    #tello.streamon()
    while not quit:
        frame = tello.get_frame_read().frame
        frame = cv2.resize(frame, (160, 120))
        frame_bytes = frame.tobytes()
        #cv2.imshow("Tello Video", frame)
        packet_size = 57600  # Adjust this value as needed
        for i in range(0, len(frame_bytes), packet_size):
            packet = frame_bytes[i:i + packet_size]
            server_socket.sendto(packet, ('192.168.0.199', 12347))
            server_socket.sendto(packet, ('192.168.0.202', 12347))
            server_socket.sendto(packet, ('192.168.0.90', 12347))
            server_socket.sendto(packet, ('192.168.0.41', 12347))
            #print(f"Sending packet of size {len(packet)} bytes")
        time.sleep(0.001)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    tello.streamoff()
    cv2.destroyAllWindows()

    #tello.streamon()

def manage_video_streaming(telloswarm):
    for tello in telloswarm:
        threading.Thread(target=video_stream, args=(tello,), daemon=True).start()

# ============================= Unity Waypoints ==============================

def handle_udp_input(sock_5007, sock_8021):
    global stop_udp_thread

    while not stop_udp_thread:
        try:
            readable, _, _ = select.select([sock_5007, sock_8021], [], [], 0.001)
            for s in readable:
                data, addr = s.recvfrom(400)  # Adjust buffer size if necessary
                if s == sock_5007:
                    BRI_command = receive_command(data)
                    if BRI_command is not None:
                        command_queue.put(BRI_command)
                if s == sock_8021:
                    updates = process_unity_data(data)
                    if updates:
                        unity_data_queue.put(updates)
        except Exception as e:
            print(f"Error handling UDP input: {e}")

    print("UDP input thread is stopping...")


def process_unity_data(data):
    # Decode and process Unity data
    message = data.decode('utf-8').strip('()')
    parts = message.split(',')
    
    if len(parts) == 36:  # Ensure there are exactly 30 parts
        vectors = []
        updates = {}
        
        for i in range(12):  # Assuming each coordinate triplet forms a vector
            # Convert meters to centimeters, float to int, and adjust the coordinate order and height check
            x = int(float(parts[i * 3]) * 100)    # x coordinate
            y = int(float(parts[i * 3 + 1]) * 100)    # y coordinate (height)
            z = int(float(parts[i * 3 + 2]) * 100)    # z coordinate

            # Ensure that negative heights are set to 0
            if y < 0:
                y = 0

            vectors.append((x, z, y))  # Store as (x, z, y) where y is height now

        # Collect only the necessary updates
        for idx, vec in enumerate(vectors[3:9], 2):  # Start from 2 to match telloMiddle index
            updates[idx] = (vec[0], vec[1], vec[2], velocity)

        return updates
    
    return None

# =========================== BCI ============================================

def send_feedback(feed_back_sock, feed_back_IP, feed_back_PORT):
    # print("sending feedback")
    MESSAGE = struct.pack('!i', 0)
    feed_back_sock.sendto(MESSAGE, (feed_back_IP, feed_back_PORT))


def receive_command(b_data):
    try:
        data = struct.unpack('!i', b_data)[0]
        print("receive target: %s" % data)
        return data

    except:
        return None

# ===========================================================================

if __name__ == '__main__':
    
    last_command_time = time.time()
    BRI_command = None
    try:
        print('========== SETTING UP Drone Swarm ==============')
        telloswarm = TelloSwarm.fromIps(["192.168.0.7"])  # 220 # 7
        telloswarm.connect()
        telloswarm.set_speed(velocity)

        for i, tello in enumerate(telloswarm):
            tello.LOGGER.setLevel(logging.ERROR)
            #print(f'Tello Battery {i+1}: {tello.get_battery()}')
            tello.change_vs_udp(11111+i)
            tello.set_video_resolution(Tello.RESOLUTION_480P)
            tello.set_video_bitrate(Tello.BITRATE_1MBPS)


        telloswarm.parallel(lambda drone, tello: tello.streamon())
        time.sleep(2)
        print('Tello Swarm is Ready!! ')
        manage_video_streaming(telloswarm)
        print('========== SETTING UP BCI ==============')
        my_ip=socket.gethostbyname(socket.gethostname())
        print('my IP: ' + str(my_ip))

        print('========== UNITY COORDINATE ============')
        udp_thread = threading.Thread(target=handle_udp_input, args=(sock_5007, sock_8021))
        udp_thread.start()
        print('Unity UDP has been setup')
        print('========================================')

        print('TAKING OFF!! ')
        currentPos = 0
        if flyingEnabled: telloswarm.takeoff()
        isFlying = True
        if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
        
        print('All Ready To Go!')
        print('----- Start Listening Commands -----')
        while not quit:
            time.sleep(1)
        # while True:
            current_time = time.time()

            # Check for new commands
            if not command_queue.empty():
                BRI_command = command_queue.get()
                print(f"Updated BRI Command in main thread: {BRI_command}")
                last_command_time = current_time  # Reset the timer each time a new command is processed
            else:
                BRI_command = None

            # Check for new unity data
            if not unity_data_queue.empty():
                updates = unity_data_queue.get()
                for idx in updates:
                    telloMiddle[idx] = updates[idx]
                    #print(f"Updated telloMiddle[{idx}] in main thread: {telloMiddle[idx]}")

            if (BRI_command != None):
                if (BRI_command <= 6 and BRI_command != 0):
                    currentPos = BRI_command

            # Periodically send the current waypoint to avoid timeout
            if BRI_command == None and isFlying == True:
                if current_time - last_command_time >= 0.5:  # More than 1 second since last command
                    if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[currentPos+1][0], telloMiddle[currentPos+1][1], telloMiddle[currentPos+1][2], telloMiddle[currentPos+1][3],1)
                    print(f"Sent keep-alive command to position {currentPos}")
                    last_command_time = current_time
            
            if BRI_command == 1 :
                if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[2][0], telloMiddle[2][1], telloMiddle[2][2], telloMiddle[2][3],1)
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
                print('--------------------------------------------------------- 1')
                print('Go to: (', telloMiddle[2][0], telloMiddle[2][1], telloMiddle[2][2], ')')

            elif BRI_command == 2 :
                if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[3][0], telloMiddle[3][1], telloMiddle[3][2], telloMiddle[3][3],1)
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
                print('--------------------------------------------------------- 2')

            elif BRI_command == 3 :
                if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[4][0], telloMiddle[4][1], telloMiddle[4][2], telloMiddle[4][3],1)
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
                print('--------------------------------------------------------- 3')

            elif BRI_command == 4 :
                if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[5][0], telloMiddle[5][1], telloMiddle[5][2], telloMiddle[5][3],1)
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
                print('--------------------------------------------------------- 4')

            elif BRI_command == 5 :
                if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[6][0], telloMiddle[6][1], telloMiddle[6][2], telloMiddle[6][3],1)
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
                print('--------------------------------------------------------- 5')

            elif BRI_command == 6 :     
                if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[7][0], telloMiddle[7][1], telloMiddle[7][2], telloMiddle[7][3],1)
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
                print('--------------------------------------------------------- 6')

            elif BRI_command == 7 :     # Middle High Position 
                if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[0][0], telloMiddle[0][1], telloMiddle[0][2], telloMiddle[0][3],1)
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
                print('--------------------------------------------------------- 7')

            elif BRI_command == 8 :     # Middle Low Position (prepare for landing)
                if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
                print('--------------------------------------------------------- 8') 

            elif BRI_command == 0 :     # When BCI got an error input, return to the middle 
                if flyingEnabled: telloswarm.go_xyz_speed_mid(telloMiddle[1][0], telloMiddle[1][1], telloMiddle[1][2], telloMiddle[1][3],1)
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
                print('Wrong Signal - Back to Origin')

            elif BRI_command == 9 :
                if flyingEnabled: telloswarm.land()
                isFlying = False
                print('LANDING')
                if sendFeedback:
                    send_feedback(feed_back_sock, feed_back_IP1, feed_back_PORT)    
                    send_feedback(feed_back_sock, feed_back_IP2, feed_back_PORT)
            
            elif BRI_command == 10 :
                if flyingEnabled: telloswarm.takeoff()
                isFlying = True
                print('Takeoff')  

            elif BRI_command == -1 :
                if flyingEnabled: telloswarm.land()
                telloswarm.end()
                stop_udp_thread = True
                udp_thread.join()
                print('Ending Program from BCI Command ... ')
                break

    except KeyboardInterrupt:
        print("Keyboard Interrupt detected, stopping threads...")
        if flyingEnabled: telloswarm.land()
        telloswarm.end()
        stop_udp_thread = True
        udp_thread.join()
        print("Threads successfully stopped.")

    except Exception as e:
        print(f"Unexpected error: {e}")