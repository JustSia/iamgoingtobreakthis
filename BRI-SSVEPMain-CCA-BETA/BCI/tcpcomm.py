import threading
import socket
import numpy as np
import atexit
import time
import sys
import struct

# from playsound import playsound
from calibrate import classifier
from calibrate_er import classifier as classifier_er

# For Streaming the marker to lsl
from tcp_to_lsl import send_marker

send_marker = send_marker()                              # <--------------- LSL EVENT 

class TCPComm(threading.Thread):

    def __init__(self, mindoobj, host="192.168.43.129", recvesize=1, srate=500, time=5,
                 savedata=True, session=1, trials=24, name="Charlie", port=8052, robohost=True, online=True):
        threading.Thread.__init__(self)
        self.received_marker = False
        self.srate = srate
        self.time = time
        self.host = host
        self.port = port
        self.robohost = robohost

        if robohost is not None:
            self.robomaster = True
        else:
            self.robomaster = False

        self.trials = trials
        assert self.trials%6==0, "Wrong number of trails".format(self.trials)

        self.setup_robot()
        self.mySocket = socket.socket()  ## Hololens /mobile - TCP
        self.mySocket.bind((self.host, self.port))
        self.mindoobj = mindoobj
        self.recvsize = recvesize
        self.gameover = False
        self.savedata = savedata
        self.session = session
        self.name = name
        self.clf = classifier(srate=srate, name=name)
        self.clf_er = classifier_er(srate=srate, name=name)

        print("init done")
        print("trail =",self.trials)
        atexit.register(self.stoptcp)
        self.n_class=len(self.clf.freqs)
        self.targets = np.arange(self.n_class)+1
        self.targets = self.targets.repeat(int(self.trials/self.n_class))

        rng = np.random.default_rng()
        rng.shuffle(self.targets)
        # self.targets =np.random.shuffle(self.targets )
        # print("targets:",self.targets)
        self.robot_reached = True

        self.online = online
        # ---------- Multi User Ver -----------------
        self.pause = False
        # -------------------------------------------

    def setup_robot(self):
        # UDP sockets
        if self.robomaster:
            self.BCI_IP = self.host  # RPI for BCI
            self.BCI_port = 5006
            self.bci_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
            # self.bci_sock.setblocking(False)
            self.bci_sock.bind((self.BCI_IP, self.BCI_port))
            print("robot access: ", self.robomaster)
            # self.robohost = self.
            self.ROBOT_IP = self.robohost  # IP address of GHOST 192.168.168.134
            
            self.DEFAULT_PORT = 5005
            # --------- NEW ---------------
            self.UNITY_PORT = 5105
            self.DRONE_PORT = 5007
            # ------------------------------
            self.ghost_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # self.robosoc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # self.robosoc.connect((self.robohost, 9999))
            print("robot connected")
        else:
            print("not using robot")

    def run(self):
        self.mySocket.listen(1)                                                                 # Here is TPC connection   <---------------------------------------------------------------
        conn, addr = self.mySocket.accept()
        self.conn = conn
        i = 0
        got="8"
        
        if self.mindoobj is not None:
            print("mindo_status: ", self.mindoobj.serstatus)
        else:
            print("No device connected. Running without EEG data.")
        
        print("------------------- Trial [{}] -------------------".format(i+1))
        order = []
        ssvep_eeg = []
        rsvp_eeg = []
        ssvep_labels = []
        er_labels=[]
        decision_times=[]
        rsvp_labels = []
        self.eeg_label=[]
        self.er_label=[]

        hololensLooked = []


        # for
        # start_time=time.time()
        # wait for the ghost to stand up

        # prepared=None
        # if self.robomaster==True:
        #     while prepared==None:
        #         b_data, addr = self.bci_sock.recvfrom(1024)  # buffer size is 1024 byte ## under waiting
        #         prepared = struct.unpack('!i', b_data)[0]
        #         print("robot is prepared: ",prepared)

        while (self.mindoobj is None or self.mindoobj.serstatus == 1) and len(er_labels) < self.trials:
            start_time = time.time()
            # produce the sound:
            if i < self.trials:
                target=self.targets[i]

            # if got not in ["7", "8"] and i < self.trials:
            if got in ["7", "8"] and i < self.trials:
                if self.robomaster==True and self.robot_reached ==True:
                    self.robot_reached = False
                    # playsound('{}.mp3'.format(target))
                # elif self.robomaster==True and self.robot_reached ==False:
                #     pass
                else:
                    print("------------------- Trial [{}] -------------------".format(i+1))
                    # print("-------- Giving instruction of target[{}] --------".format(target))
                    # playsound('{}.mp3'.format(target))
            # print("mindo_status: ", self.mindoobj.serstatus)
            data = self.conn.recv(self.recvsize)  ## input from hololens
            data = self.conn.recv(self.recvsize)  ## input from hololens
            gap_time = time.time() - start_time

            if not data:
                self.end_loop()
                break
            got = data.decode()
            # print(got, type(got))

            if data.decode() == "s":
                print("Stopping System")
                self.end_loop()
            elif data.decode() == "p":                                                                               
                print("SSVEP BCI Paused")
                print("------------- Abandoning Trial [{}] --------------".format(i+1))
                print("SSVEP BCI output:  N/A")
                ssvep_labels.append(10)
                self.eeg_label.append(10)
                self.start_pause()    
            elif data.decode() == "g":      
                i+=1 
                er_labels.append(8)
                self.er_label.append(2)
                self.send_unpause(i)
                                                                                    
            elif data.decode() in ["1", "2", "3", "4", "5", "6", "7", "8"]:
                if self.pause == False:
                    # eeg_signal = self.mindoobj.eeg_final[[2, 3, 5, 4], :]
                    # eeg_signal = self.mindoobj.eeg_final[[0, 1, 2, 3, 4, 5], :]
                    eeg_signal = self.mindoobj.eeg_final

                    # print("HoloLens User Looking at: ", got)

                    if got in ["1", "2", "3", "4", "5", "6"]:
                        ssvep_eeg.append(eeg_signal)
                        ssvep_labels.append(int(got))
                    elif got in ["7", "8"] and i!=0:                                                                            #    Maybe change the new yes no flickers to  9, 10
                        ssvep_eeg.append(eeg_signal)
                        er_labels.append(int(got))
                    self.send_decision(eeg_signal, got)  # for classification and ghost feedback

            if got in ["7", "8"]:
                # print("HoloLens User ER: ", got)
                i+=1
                self.robot_reached = False

            # gap_time = time.time() - start_time
            # start_time = time.time()
            decision_times.append(gap_time)

        if self.savedata:
            pass
            # np.save("ssvep_data_{}_session_{}.npy".format(self.name, self.session), np.array(ssvep_eeg))
            # np.save("ssvep_labels_{}_session_{}.npy".format(self.name, self.session), np.array(ssvep_labels))
        # end_time = time.time()

        #print("Target_outputs =", self.targets[:len(self.eeg_label)])   #Original
        print("Target_outputs =", ssvep_labels)
        print("SSVEP_outputs = ", self.eeg_label)
        #print("Decision times = ", decision_times)

        # match= self.targets[:len(self.er_label)] == self.eeg_label
        # Convert lists to NumPy arrays
        ssvep_labels_array = np.array(ssvep_labels)
        eeg_label_array = np.array(self.eeg_label)

        # Compare arrays element-wise and calculate accuracy
        match = ssvep_labels_array == eeg_label_array
        accuracy = match.mean()
        print("Accuracy:", accuracy)
        # print("accuracy =",accuracy)
        er_true=match*1+1
        er_match = er_true[:len(self.er_label)] == self.er_label
        er_accuracy = (er_match * 1).mean()
        # print("er_accuracy =", er_accuracy)
        itr=self.itr(len(self.eeg_label), accuracy, 4)

        print("6.ITR = {} bits/min".format(str(itr)[:5]))
        print("7.SSVEP Accuracy = {}%".format(accuracy*100))
        print("8.ER Accuracy = {}%".format(er_accuracy*100))
        print("9.Time = {}(s)".format(4*len(self.eeg_label)))

        if self.mindoobj.serstatus == 0: 
            print("Not Connected to HoloLens or")
            print("EEG Device Not Found, Please check LSL or other connection types")

        time.sleep(0.1)
        self.mindoobj.serstatus = 0
        self.stoptcp()
        time.sleep(0.1)
        print("tcp exit")

    def end_loop(self):
        self.mindoobj.serstatus = 0
        self.mindoobj.streaming = False
        self.received_marker = True
        self.gameover = True

    def start_pause(self):                                                                                                     # <-------------------  Finish this part
        self.pause = True

    def stoptcp(self):
        try:
            self.mindoobj.stop_streaming()
        except:
            pass

        try:
            self.conn.close()
        except:
            pass
        
        self.mySocket.close()

        if self.robomaster:
            self.ghost_sock.close()
            self.bci_sock.close()

    def sendmsgtohololens(self, msg):
        self.conn.send(msg.encode())

    def get_ssvepcls(self, eeg_signal):
        cls = self.clf.get_ssvep_command(eeg_signal)
        return cls

    def get_er_cls(self, eeg_signal):
        cls = self.clf_er.get_ssvep_command(eeg_signal)
        return cls

    def pack_and_send(self, cmd):
        Leo_IP = "192.168.0.98"
        Tello_IP = "192.168.0.202"

        Send_IP2 = "192.168.0.2"
        BRIPC_IP = "192.168.0.202"
        DellPC_IP = "192.168.0.183"

        Phone_IP = "192.168.0.109"   # For Testing on Phone AR

        MESSAGE = struct.pack('!i', cmd)

        # ROBOHOST to All Three Ports
        self.ghost_sock.sendto(MESSAGE, (self.ROBOT_IP, self.DEFAULT_PORT))  # <----- robothost , PORT: 5005
        self.ghost_sock.sendto(MESSAGE, (self.ROBOT_IP, self.DRONE_PORT))    # <----- robothost , PORT: 5007
        self.ghost_sock.sendto(MESSAGE, (self.ROBOT_IP, self.UNITY_PORT))    # <----- robothost , PORT: 5105

        if self.ROBOT_IP != self.host:
            self.ghost_sock.sendto(MESSAGE, (self.host, self.DEFAULT_PORT))  # <----- robothost , PORT: 5005
            self.ghost_sock.sendto(MESSAGE, (self.host, self.DRONE_PORT))    # <----- robothost , PORT: 5007
            self.ghost_sock.sendto(MESSAGE, (self.host, self.UNITY_PORT))    # <----- robothost , PORT: 5105

        # Manual Ip Settings
        ## Leo:
        if self.ROBOT_IP != Leo_IP and self.host != Leo_IP:
            self.ghost_sock.sendto(MESSAGE, (Leo_IP, self.DEFAULT_PORT))           # <----- Liang's IP for LEO   PORT:5005  
        ## Tello Drone:
        if self.ROBOT_IP != Tello_IP and self.host != Tello_IP:
            self.ghost_sock.sendto(MESSAGE, (Tello_IP, self.DRONE_PORT))
        ## NReal Virtuals Drone:
        self.ghost_sock.sendto(MESSAGE, (Phone_IP, self.UNITY_PORT)) 

    def send_decision(self, eeg_signal, got):
        peak = False
        if got in ["7", "8"]:
            self.send_ghost_er_cmd(eeg_signal, got)
        elif got in ["1", "2", "3", "4", "5", "6"]:
            self.send_ghost_cmd(eeg_signal, got)

    def send_unpause(self, i):
        # Send event by LSL
        # send_marker.send("unpause")                           # <--------------- LSL EVENT 
        # cls = int(input("enter ssvep cmd: "))
        self.pause = False
        self.robot_reached=True
        print("SSVEP BCI Unpaused")
        print("------------------- Trial [{}] -------------------".format(i+1))
        self.sendmsgtohololens("10")  # to start new trial         # is 9 better or g or 10    9 is er = no, restart the trial

    def send_ghost_cmd(self, eeg_signal, got):
        cls = self.get_ssvepcls(eeg_signal)  # command
        # Send event by LSL
        
        send_marker.send(str(cls + 1))                           # <--------------- LSL EVENT 
        # cls = int(input("enter ssvep cmd: "))
        print("SSVEP BCI output: ", cls + 1)
        self.eeg_label.append(cls + 1)
        if self.robomaster:
            if self.online == True:
                self.pack_and_send(cls + 1)
            else:
                self.pack_and_send(int(got))
                print("Got HoloLens output: ", got)
        self.sendmsgtohololens(str(cls + 1))  # to start new trial


    def send_ghost_er_cmd(self, eeg_signal, got):
        peak = False
        # cls = self.get_er_cls(eeg_signal)  # Original Command
        cls = 1                       # ------------------------ TO TURN OFF ER  ----------------------------------
       
        ## cls = int(input("enter ER cmd: "))   # (Testing Purpose only)
        if cls ==0:
            print("ER output: 1 (incorrect-STOP)")
        elif cls==1:
            print("ER output: 2 (correct)")
        self.er_label.append(cls + 1)

        if self.robomaster:
            if cls == 0:
                self.pack_and_send(cls)
            else:
                pass
            b_data, addr = self.bci_sock.recvfrom(1024)  # buffer size is 1024 byte ## under waiting
            data = struct.unpack('!i', b_data)[0]
            print("Received Feedback from Drone: %s" % data)
            self.robot_reached=True
        self.sendmsgtohololens(str(cls + 1))  # to start new trial


    def itr(self,n, p, t):
        """Compute information transfer rate (ITR).

        Definition in [1]_.

        Parameters
        ----------
        n : int
            Number of targets.
        p : float
            Target identification accuracy (0 <= p <= 1).
        t : float
            Average time for a selection (s).

        Returns
        -------
        itr : float
            Information transfer rate [bits/min]

        References
        ----------
        .. [1] M. Cheng, X. Gao, S. Gao, and D. Xu,
            "Design and Implementation of a Brain-Computer Interface With High
            Transfer Rates", IEEE Trans. Biomed. Eng. 49, 1181-1186, 2002.

        """
        itr = 0

        if (p < 0 or 1 < p):
            # raise ValueError('Accuracy need to be between 0 and 1.')
            print('Accuracy need to be between 0 and 1.')
        elif (p < 1 / n):
            itr = 0
            # raise ValueError('ITR might be incorrect because accuracy < chance')
            print('ITR might be incorrect because accuracy < chance')
        elif (p == 1):
            itr = np.log2(n) * 60 / t
        else:
            itr = (np.log2(n) + p * np.log2(p) + (1 - p) *
                   np.log2((1 - p) / (n - 1))) * 60 / t

        return itr