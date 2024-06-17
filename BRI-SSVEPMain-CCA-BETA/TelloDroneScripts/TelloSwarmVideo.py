# === MAYBE WORKING VIDEO STREAMING FROM ROUTER IP in SWARM MODE ===
# By Justin
# Date: 18/04/2024
# USE TEST Python venv

from djitellopy import Tello, TelloSwarm
import cv2
import threading
import time, logging
import socket
import struct

flip = False
fly = False
video = True
landed = False 
displayVideo = True
quit = False
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
            server_socket.sendto(packet, ('192.168.0.202', 12347))
            print(f"Sending packet of size {len(packet)} bytes")
            time.sleep(0.001)
        if cv2.waitKey(40) & 0xFF == ord('q'):
            break

    server_socket.close()
    cv2.destroyAllWindows()


def main():
    global quit
    tello_swarm = TelloSwarm.fromIps(['192.168.0.7'])  # Adjust IPs as needed
    tello_swarm.connect()
    print(f"Connected to TelloSwarm with {len(tello_swarm)} drones")

# not tested yet.......
    for i, tello in zip(range(3), tello_swarm):
        tello.LOGGER.setLevel(logging.ERROR) 
        tello.connect()
        #print(f'Tello Battery {i+1}: {tello.get_battery()}')
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
