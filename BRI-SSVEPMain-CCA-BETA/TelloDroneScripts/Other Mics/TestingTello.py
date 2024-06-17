# NO STREAM WORKING FOR AP Mode

from djitellopy import TelloSwarm
import time
import cv2

velocity = 100

if __name__ == '__main__':

    print('========== SETTING UP Drone Swarm ==============')
    telloswarm = TelloSwarm.fromIps(["192.168.0.220"])
    telloswarm.connect()
    telloswarm.set_speed(velocity)
    telloswarm.streamon()
    print('Tello Swarm is Ready!! ')

    print('All Ready To Go!')
    print('----- Start Listening Commands -----')

    try:
        while True:
            print('do stuff')
            cv2.waitKey(1)
            img = telloswarm.get_frame_read().frame
            img = cv2.resize(img, (960, 720))
            im_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            cv2.waitKey(1)
            cv2.imshow("Image", im_rgb)
            cv2.waitKey(1)

    except KeyboardInterrupt:   
        print("Keyboard Interruptted!  Closing Thread.... ")
        telloswarm.end()
