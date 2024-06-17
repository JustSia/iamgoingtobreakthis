# === WORKING VIDEO STREAMING and KEYBOARD CONTROL FROM ROUTER IP in SWARM MODE ===
# By Charlie
# Date: 10/Mar/2024
# USE TEST Python venv

from djitellopy import Tello, TelloSwarm
import cv2
import threading
import time
import logging
from pynput import keyboard
import socket

# Global variables
quit = False
video = True
commands = []  # Shared data structure for keyboard commands

#===================== Display Video in Local Machine Window (WORKING) ======================
# # Video streaming function for each drone
# def tello_video(tello, drone_number):
#     global quit
#     while not quit:
#         frame = tello.get_frame_read().frame
#         frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         cv2.imshow(f'Tello {drone_number}', frame)
#         cv2.moveWindow(f'Tello {drone_number}', (drone_number - 1) * 900, 50)
#         if cv2.waitKey(40) & 0xFF == ord('q'):
#             break
#     cv2.destroyAllWindows()
# =================================================================================
    



def tello_video(tello, drone_number):
    global quit
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(('0.0.0.0', 12346))  # Bind to an address and port


    while not quit:
        frame = tello.get_frame_read().frame
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (160, 120))
        # Convert the frame to bytes and send it to the Unity client
        frame_bytes = frame.tobytes()

        #cv2.imshow(f'Tello {drone_number}', frame)

        # Split the frame into smaller packets
        packet_size = 57600  # Adjust this value as needed
        for i in range(0, len(frame_bytes), packet_size):
            packet = frame_bytes[i:i + packet_size]
            server_socket.sendto(packet, ('192.168.0.90', 12347))
            print(f"Sending packet of size {len(packet)} bytes")
            time.sleep(0.001)
        if cv2.waitKey(40) & 0xFF == ord('q'):
            break

    server_socket.close()
    cv2.destroyAllWindows()


# =====================================================================================




# Function to handle keyboard inputs
def keyboard_listener():
    global quit, commands
    def on_press(key):
        try:
            if key.char not in commands:
                commands.append(key.char)
        except AttributeError:
            pass
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    listener.join()

# Function to control the drone based on keyboard inputs
def drone_control(tello_swarm):
    global quit, commands
    distance = 50
    ang_velocity = 45
    while not quit:
        if commands:
            command = commands.pop(0)
            if command == 'l':
                tello_swarm.land()
            elif command == 't':
                tello_swarm.takeoff()
            elif command == 'w':
                tello_swarm.move_forward(distance)
            elif command == 's':
                tello_swarm.move_back(distance)
            elif command == 'a':
                tello_swarm.move_left(distance)
            elif command == 'd':
                tello_swarm.move_right(distance)
            elif command == '.':
                tello_swarm.move_up(distance)
            elif command == ',':
                tello_swarm.move_down(distance)
            elif command == 'e':
                tello_swarm.rotate_clockwise(ang_velocity)
            elif command == 'q':
                tello_swarm.rotate_counter_clockwise(ang_velocity)
            elif command == 'u':
                tello_swarm.flip_forward()
            elif command == 'j':
                tello_swarm.flip_back()
            elif command == 'h':
                tello_swarm.flip_left()
            elif command == 'k':
                tello_swarm.flip_right()
            elif command == 'p':
                print('Pressed P, Killing Motors!')
                quit = True
            # Add more commands as needed
        time.sleep(0.1)

def main():
    global quit
    tello_swarm = TelloSwarm.fromIps(['192.168.0.7'])  # Adjust IPs as needed
    tello_swarm.connect()
    print(f"Connected to TelloSwarm with {len(tello_swarm)} drones")

# not tested yet.......
    for i, tello in zip(range(3), tello_swarm):
        tello.LOGGER.setLevel(logging.ERROR) 
        tello.connect()
        print(f'Tello Battery {i+1}: {tello.get_battery()}')
        tello.change_vs_udp(11111+i)
        tello.set_video_resolution(Tello.RESOLUTION_480P)
        tello.set_video_bitrate(Tello.BITRATE_1MBPS)

    try:
        if video:
            tello_swarm.parallel(lambda drone, tello: tello.streamon())
            time.sleep(2)  # Wait for video stream to stabilize

            # Start video streaming threads
            for i in range(len(tello_swarm)):
                threading.Thread(target=tello_video, args=(tello_swarm.tellos[i], i+1), daemon=True).start()
                print(f'Tello{i+1} Video Started')

        # Start keyboard listener thread
        threading.Thread(target=keyboard_listener, daemon=True).start()
        # Start drone control thread
        threading.Thread(target=drone_control, args=(tello_swarm,), daemon=True).start()

        # Keep the main thread alive until landing
        while not quit:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Terminating the program...")
        quit = True
        tello_swarm.parallel(lambda drone, tello: tello.streamoff())  # Turn off video streaming
        tello_swarm.end()  # Properly disconnect all drones

if __name__ == '__main__':
    main()
