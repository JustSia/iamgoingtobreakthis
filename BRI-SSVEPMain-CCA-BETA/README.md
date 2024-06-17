# BRI SSVEPMain ver.CCA
<a name="readme-top"></a>
# HAI Centre Augmented Reality (AR) Enabled Brain-Computer Interface (BCI) Drone Swarm Control System
### **With Crazyflie 2.1 Mini Drones and HTC Vive VR Base Stations**

![BCI Multiplayer](https://github.com/CharlieeT/BCIDroneSwarm/assets/88194090/f5e34890-fa2d-4239-b9af-a173fb468769)


## Project Description
This repo is intended for the UTS HAI BRI Team. Drone Swarm BCI Demo which uses the Lighthouse base stations or OptiTrack Mocap as external positioning system. 

The main focus of this project is to implement Brain-Compiuter Interface (BCI) as the control method for the Crazyflie drone swarm, which uses the combination of a Microsoft Hololens 2.0 and a custom-made dry electrode array to read the EEG brainwave signals of the user to control the movement of the drones while an external positioning system is used to provide accurate tracking of the position and orientation for each individual drones in real-time.


## Software Setup and Installation Guide
### System:
1. Windows 10 or 11
2. Python Version: 3.8.0 or above (3.10.0 Recommended)
3. :warning: **Python 3 (64 Bits)**:warning:

### Python Environment Setup and Required Packages Installation Guide: 
1. Install Python3 using the official python build from python.org. Make sure to check the “add to path” checkbox during install. Make sure to check if it works by opening a cmd or powershell terminal:
```sh
python --version
python -m pip install --upgrade pip
pip --version
```

2. Install git from the official git website. Make sure it is in PATH as well, which can be checked also with:
```sh
git --version
``` 

3. Upgrade the pip to the latest version:
```sh
pip3 install --upgrade pip
```

4. Download all of the files in this git repository by typing:
```sh 
git clone https://github.com/CharlieeT/BCIDroneSwarm.git
```

5. Create a Python Virtual Enviornment for Package installation:
```sh
python -m venv BRI
```

6. Activate the BRI Python venv (For Windows):
 ```sh
BRI\Scripts\activate.bat
```

7. Install the necessary pip packages by using the following commands:
```sh
cd .[PATH TO THIS FOLDER]/BCIDroneSwarm
pip install -r "bri_requirements.txt"
```

### Drone Radio Driver for Windows:
   1. Install the CrazyRadio Driver by following the link below and follow the images below for the correct installation:
   http://zadig.akeo.ie/
   
   ![02-zadig_setup](https://user-images.githubusercontent.com/88194090/231747490-c60af588-cb3f-4be3-a80b-07bd4ab38a03.png)


### Brain-Computer Interface Related Software for Windows:
   1. Install the MentaLab software using the ExploreDesktopInstaller10.exe (with version 0.2.3) for Windows 10 and ExploreDesktopInstaller11.exe for Windows 11.



## Hardware Setup


### Microsoft Hololens and BCI Setup: 

For HoloLens related software interfaces, please check the other repo.
https://github.com/CharlieeT/UnityDroneSwarm


### Crazyflie Drone Setup in Lighthouse Mode:
1. Equip all three of the Crazyflie 2.1 Drones with the Lighthouse Deck like such

![Crazyflie-LH](https://user-images.githubusercontent.com/88194090/231730940-4adcfa2a-f44f-448d-bbd7-495f6bc477e3.jpg)

2. Install the HTC VIVE Lighthouse Base Station (minimum of 2) at the corner of the flying zone

![Virtual Envrio](https://user-images.githubusercontent.com/88194090/231744545-d445f309-6a0d-4238-832c-7bebc4524cde.png)

3. Place the three drones in the position and orientation as indicated in the picture below (Front facing the Motion Platform) 

![Drone Loca](https://user-images.githubusercontent.com/88194090/231741299-61ed5d26-869f-4245-bc22-3193a229afb4.jpg)

4. Open the Crazyflie Client via this command:
```sh 
cfclient
```

5. Randomly pick a drone with the Lighthouse Deck installed, place it at the origin of the flying zone and connect it with the client app.

6. Go to the Lighthouse Positioning Tab. (You may have to check it in the menu View -> Tabs -> Lighthouse Positioning Tab to make it visible.)
![1_client_lighthouse_tab](https://user-images.githubusercontent.com/88194090/231751919-3a554683-2999-4e27-b52d-d13f0e00c6d3.png)

7. To start the calibration process, Press the Manage Geometry button and then click on Estimate Geometry on the pop-up window like such:
![5_geometry_dialog](https://user-images.githubusercontent.com/88194090/231752014-f85399ea-8d7b-4fa2-ba8d-1a315588840b.png)

8. Follow the instruction of the client to proceed with the calibration of lighthouse sensors and use this image as a guide for the calibration placement of the drone:
![Virtual Envrio 2](https://user-images.githubusercontent.com/88194090/231751197-f737c0b7-6ad5-40cf-a34e-a0c34bd27a43.png)

9. At step 4, set the calibration duration to over 90 seconds and start the wanding process by taking the drone and moving it around the flying zone. 
   - Make sure to cover as many of the flying space as possible. Especially the position where the drone is more likely to be at. Ex: The Waypoints and Takeoff Zone.

10. When the calibration results are displayed, REMEMBER to press "Write to Crazyflie" and close the window. 

11. Click "Save System Config" to save the calibration data for the rest of the drones.

12. Connect the next drone with the client and click on "Load System Config" to pass on the calibration data. Repeat until all of the drones are calibrated.

13. If the calibration process is done properly, the base station status will display all green for all four base stations.


### Crazyflie Drone Setup in Mocap Mode:

![1](https://user-images.githubusercontent.com/88194090/231739112-202d00c3-9984-4134-80cd-fc7aa2fbee5f.png)

1. The crazyflie drones that are used for the mocap setup are C10 & C20. 

2. Turn on the HP PC in UTS TechLab

3. Connect the OptiTrack Camera Link Cables (There are two) to the PC as well as the Licence Key USB Drive.

4. Run the Motive 2.1.2 software.

5. Calibrate the mocap system by using the CW-500 Wand, place and orient the ground plane so that X is pointing towards the Motion Platform.

6. Place the two drones in the capture area with C10 at the origin and C20 closer towards the computers, both facing the Motion Platform. (See the MocapSwarmBCIControl.py for more information)

7. Create the ridge body for both drones and make sure that the yaw, pitch, roll are all at 0 degrees.

8. Go to the straaming tab and enable streaming to local network. (NOT LOCALHOST or LOOPBACK)

9. Change the IP in the NatNetClient.py (inside the mocap folder) to match with the Mocap host PC and your client PC.

10. Run the MocapSwarmBCIControl.py and the final stage for maximum fun!



## USAGE: 

### MentaLab Pairing and Connection ###
1. Launch the App: Explore Desktop, and Scan and Connect with the device.
![image](https://github.com/CharlieeT/BCIDroneSwarm/assets/88194090/40a967aa-b933-47a8-8de6-78b9faed9122)

2. Check the Impedance Measurement by switching the mode to "Dry Electrodes" and make sure the impedance value for each channel is less than 150 kΩ.
![Screenshot 2023-10-06 171031](https://github.com/CharlieeT/BCIDroneSwarm/assets/88194090/3a0eedbf-27ff-4448-b4ec-0889ed2bb3d6)

3. Change the Sampling Rate from 250 to 500.
![image](https://github.com/CharlieeT/BCIDroneSwarm/assets/88194090/891390cb-d498-403d-b0a9-4c25cb152e6e)

4. Push the EEG Data to Lab Streaming Layer (LSL):
![image](https://github.com/CharlieeT/BCIDroneSwarm/assets/88194090/64ef8ce6-6112-456e-b162-a275101b2827)


### Starting the BCI Main Script using Terminal:

:warning: **MAKE SURE TO CONNECT TO MENTALAB BEFORE LAUNCHING THE SCRIPT AND HOLOLENS APP!!!** :warning:

1. To Connect the HoloLens and MentaLab using script

  **a. With Real Robots (Feedback ON):**
      - Replace the ip address with the current local ip of the PC that will be running the script.
   ```sh
   python RunBCI.py --device menta --robhost 192.168.0.202 --trial 24 
   ```
   ![Screenshot 2023-04-13 223116](https://user-images.githubusercontent.com/88194090/231759046-eae67ce8-f59d-4c54-8c70-912fc675077d.png)

   **b. With Virtual Drones and/or Crazyflie Drone Swarm (Feedback OFF):**
   ```sh
   python RunBCI.py --device menta --trial 24 
   ```


### Starting the BCI Main Script using GUI:
1. To Connect the HoloLens and MentaLab using GUI

  **a. With Real Robots (Feedback ON):**
  Add the IP of the PC that has the robot control script. 
   - Connect with Leo Rover, add "192.168.0.98:**5005**"
   - Connect with Drones (Both Crazyflie and Tello in 6 location mode), add "192.168.0.202:**5007**"
   - Connect with Phone/Unity, add "192.168.0.109:**5105**"
  
 ![image](https://github.com/CharlieeT/BCIDroneSwarm/assets/88194090/c8648832-f191-4c20-a0db-21e4741c0ee7)

  
  **b. With Virtual Drones and/or Crazyflie Drone Swarm (Feedback OFF):**
  Leave the Ip and Port section **EMPTY** and press the Start Button
  ![Screenshot 2023-10-05 173421](https://github.com/CharlieeT/BCIDroneSwarm/assets/88194090/2bcc1c64-df1e-42fb-bbc8-3a26580047a4)


### After Starting the BCI Script or GUI:
1. After the "RunBCI.py" started, in the HoloLens, type the ip address of the host PC inside HoloLens in the setup scene.

2. After the flickers are set in the HoloLens and the BCI script is ready, run the drone control script of your choosing:
   - Before proceeding with the drone script, MAKE SURE that the "mindo_status = 1" is displayed!

3. Start the Drone Control Script of your choosing:
   - **Virtual and Real Drone Mode (3 Drones Only):**
        To Control the Crazyflie drones by following the trajectory of the virtual drones in the AR Scenario, the script will need to be launch **after** the setting of flickers but **before** pressing start button:
      ```sh 
      python '.\DroneScripts\HoloLens\UnitySwarmControl3.py'
      ```
    
   - **Six Preset Location Mode (3 Crazyflie):**
        To Control the Crazyflie drones to 6 preset location, the script "LightHouseSwarmBCIControlDoubleRow.py" will need to be launch around the same time as RunBCI.py (The sequence of starting the scripts does not matter in this case)
      ```sh 
      python '.\DroneScripts\LightHouseSwarmBCIControlDoubleRow.py'
      ```

   - **Six Preset Location Mode (Single Tello):**
        To Control one Tello drone to 6 preset location: First you will need to ping the Tello to make sure the drone is connectable.
        Charlie's Tello: 192.168.0.220
        Eric's Tello:    192.168.0.7
      ```sh 
      ping 192.168.0.220
      ```
      ![image](https://github.com/CharlieeT/BCIDroneSwarm/assets/88194090/b1b2a164-b189-420d-863b-eddae6f269b0)

      Secondly, Use a software such as Packet Sender to send a single command to Tello to ensure connection like such, and wait for the Tello to return an 'ok' and blinks purple light:
      ![image](https://github.com/CharlieeT/BCIDroneSwarm/assets/88194090/ff822705-56a2-45c2-bc1e-aac29a4e2b6c)

      Finally, launch the Control Script as followed (This will take off immediately):
      ```sh 
      python '.\TelloDroneScripts\BCIControlSingleTello.py'
      ```
       
   - **Tello and Leo Rover Collaboration:**
        To Control Both the Leo Rover and Tello Drone at the same time:
        Do everything as the previous steps and consult Liang for the Leo Rover side of control.

      Place the Drone on top of the Leo and Launch the Control Script as followed:
      ```sh 
      python '.\TelloDroneScripts\BCITelloLeoCollab.py'
      ```

5. Start commanding the robots with your MIND!

   ![me](https://github.com/CharlieeT/BCIDroneSwarm/blob/main/cache/GIF%20Charlie%20BCI.gif)
