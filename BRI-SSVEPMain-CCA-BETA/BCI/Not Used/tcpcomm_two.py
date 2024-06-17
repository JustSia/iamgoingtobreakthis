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
                 savedata=True, session=1, trials=24, name="Charlie", port=8052, robohost=True, usercount=1):
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
        assert self.trials%6==0, "wrong number of trails".format(self.trials)

        self.setup_robot()
        self.mySocket = socket.socket()  ## Hololens / mobile - TCP
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

        # ---------- Multi User Ver -----------------
        self.user = usercount
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
            self.GHOST_IP = self.robohost  # IP address of GHOST 192.168.168.134
            
            self.GHOST_PORT = 5005
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
        print("mindo_status: ", self.mindoobj.serstatus)
        order = []
        ssvep_eeg = []
        rsvp_eeg = []
        ssvep_labels = []
        er_labels=[]
        decision_times=[]
        rsvp_labels = []
        self.eeg_label=[]
        self.er_label=[]

        i = 0
        got="8"

        # for
        # start_time=time.time()
        # wait for the ghost to stand up

        # prepared=None
        # if self.robomaster==True:
        #     while prepared==None:
        #         b_data, addr = self.bci_sock.recvfrom(1024)  # buffer size is 1024 byte ## under waiting
        #         prepared = struct.unpack('!i', b_data)[0]
        #         print("robot is prepared: ",prepared)

        while self.mindoobj.serstatus == 1 and len(er_labels)<self.trials:
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
                    print("-------- Giving instruction of target[{}] --------".format(target))
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
                print("stopping..")
                self.end_loop()
            elif data.decode() == "p":                                                                                       #     Add Sending P from HoloLens 
                print("pausing...")
                #self.start_pause()                                                                                          # <-------------------  Finish this part
            else:
                # eeg_signal = self.mindoobj.eeg_final[[2, 3, 5, 4], :]
                # eeg_signal = self.mindoobj.eeg_final[[0, 1, 2, 3, 4, 5], :]
                eeg_signal = self.mindoobj.eeg_final

                if got in ["1", "2", "3", "4", "5", "6"]:
                    ssvep_eeg.append(eeg_signal)
                    ssvep_labels.append(int(got))
                elif got in ["7", "8"] and i!=0:                                                                            #    Maybe change the new yes no flickers to  9, 10
                    ssvep_eeg.append(eeg_signal)
                    er_labels.append(int(got))
                self.send_decision(eeg_signal, got)  # for classification and ghost feedback

            if got in ["7", "8"]:
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
        print("Target_outputs =", self.targets[:len(self.eeg_label)])
        print("SSVEP_outputs = ", self.eeg_label)
        # print("Decision times = ", decision_times)

        match= self.targets[:len(self.er_label)] == self.eeg_label
        accuracy = (match*1).mean()
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
        #..... add more 


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
        Phone_IP = "192.168.0.109"   # For Testing on Phone AR
        MESSAGE = struct.pack('!i', cmd)
        self.ghost_sock.sendto(MESSAGE, (self.GHOST_IP, self.DRONE_PORT))    # <----- SelfIP , PORT: 5007
        self.ghost_sock.sendto(MESSAGE, (self.GHOST_IP, self.UNITY_PORT))    # <----- SelfIP , PORT: 5005
        self.ghost_sock.sendto(MESSAGE, (Leo_IP, self.GHOST_PORT))           # <----- Liang's IP for LEO   PORT:5005  
        self.ghost_sock.sendto(MESSAGE, (Phone_IP, self.UNITY_PORT))   
        # print('sending {!r}'.format(MESSAGE))


    def send_decision(self, eeg_signal, got):
        peak = False
        if got in ["7", "8"]:
            self.send_ghost_er_cmd(eeg_signal)
        else:
            self.send_ghost_cmd(eeg_signal)


    def send_ghost_cmd(self, eeg_signal):
        cls = self.get_ssvepcls(eeg_signal)  # command
        # Send event by LSL
        
        send_marker.send(str(cls + 1))                           # <--------------- LSL EVENT 
        # cls = int(input("enter ssvep cmd: "))
        print("SSVEP BCI output: ", cls + 1)
        self.eeg_label.append(cls + 1)
        if self.robomaster:
            self.pack_and_send(cls + 1)
        self.sendmsgtohololens(str(cls + 1))  # to start new trial


    def send_ghost_er_cmd(self, eeg_signal):
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