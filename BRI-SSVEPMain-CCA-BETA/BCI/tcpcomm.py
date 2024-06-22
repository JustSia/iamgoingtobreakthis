'''
File: tcpcomm.py
Current version done by: Justin Sia
Last Modified: June 22, 2024
Stage: Tested in Simulation, Not yet tested on Holones or Robot
'''

import threading, socket, numpy as np, atexit, time, sys, struct
from calibrate import classifier
from calibrate_er import classifier as classifier_er
from tcp_to_lsl import send_marker

send_marker = send_marker()

class TCPComm(threading.Thread):
    def __init__(self, mindoobj, host="192.168.43.129", recvesize=1, srate=500, time=5, savedata=True, session=1, trials=24, name="Charlie", port=8052, robohost=True, online=True):
        threading.Thread.__init__(self)

        # Initialise parameters and object
        self.received_marker, self.srate, self.time, self.host, self.port, self.robohost, self.robomaster, self.trials, self.mySocket, self.mindoobj, self.recvsize, self.gameover, self.savedata, self.session, self.name, self.clf, self.clf_er, self.robot_reached, self.online, self.pause = False, srate, time, host, port, robohost, bool(robohost), trials, socket.socket(), mindoobj, recvesize, False, savedata, session, name, classifier(srate=srate, name=name), classifier_er(srate=srate, name=name), True, online, False
        assert self.trials%6==0, "Wrong number of trails".format(self.trials)
        self.setup_robot()
        self.mySocket.bind((self.host, self.port))
        print("init done"); print("trail =",self.trials)
        atexit.register(self.stoptcp)

        # Set up targets for trials     
        self.n_class=len(self.clf.freqs)
        self.targets = np.arange(self.n_class)+1
        self.targets = self.targets.repeat(int(self.trials/self.n_class))
        rng = np.random.default_rng(); rng.shuffle(self.targets)

    # Set up robot communication if enabled (Currently not in use)
    def setup_robot(self):
        if self.robomaster:
            self.BCI_IP, self.BCI_port, self.bci_sock, self.ROBOT_IP, self.DEFAULT_PORT, self.UNITY_PORT, self.DRONE_PORT, self.ghost_sock = self.host, 5006, socket.socket(socket.AF_INET, socket.SOCK_DGRAM), self.robohost, 5005, 5105, 5007, socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.bci_sock.bind((self.BCI_IP, self.BCI_port))
            print("robot access: ", self.robomaster)
            print("robot connected")
        else:
            print("not using robot")


    # Main loop
    def run(self):
        self.mySocket.listen(1)
        conn, addr = self.mySocket.accept()
        self.conn = conn
        i, got = 0, "8"
        print("mindo_status:", self.mindoobj.serstatus)
        print("------------------- Trial [{}] -------------------".format(i+1))
        order = ssvep_eeg = rsvp_eeg = ssvep_labels = er_labels = decision_times = rsvp_labels = self.eeg_label = self.er_label = hololensLooked = []
        while self.mindoobj.serstatus == 1 and len(er_labels) < self.trials:
            start_time = time.time()
            if i < self.trials:
                target = self.targets[i]
            if got in ["7", "8"] and i < self.trials:
                if not (self.robomaster and self.robot_reached):
                    print("------------------- Trial [{}] -------------------".format(i+1))
                else:
                    self.robot_reached = False
            data = self.conn.recv(self.recvsize)
            data = self.conn.recv(self.recvsize)
            gap_time = time.time() - start_time
            if not data:
                self.end_loop()
                break
            got = data.decode()
            if got == "s":
                print("Stopping System")
                self.end_loop()
            elif got == "p":
                print("SSVEP BCI Paused")
                print("------------- Abandoning Trial [{}] --------------".format(i+1))
                print("SSVEP BCI output:  N/A")
                ssvep_labels.append(10)
                self.eeg_label.append(10)
                self.start_pause()
            elif got == "g":
                i += 1
                er_labels.append(8)
                self.er_label.append(2)
                self.send_unpause(i)
            elif got in ["1", "2", "3", "4", "5", "6", "7", "8"] and not self.pause:
                eeg_signal = self.mindoobj.eeg_final
                if got in ["1", "2", "3", "4", "5", "6"]:
                    ssvep_eeg.append(eeg_signal)
                    ssvep_labels.append(int(got))
                elif got in ["7", "8"] and i != 0:
                    ssvep_eeg.append(eeg_signal)
                    er_labels.append(int(got))
                self.send_decision(eeg_signal, got)
            if got in ["7", "8"]:
                i += 1
                self.robot_reached = False
            decision_times.append(gap_time)
        print("Target_outputs =", ssvep_labels)
        print("SSVEP_outputs =", self.eeg_label)
        ssvep_labels_array, eeg_label_array = np.array(ssvep_labels), np.array(self.eeg_label)
        match, accuracy = ssvep_labels_array == eeg_label_array, (ssvep_labels_array == eeg_label_array).mean()
        print("Accuracy:", accuracy)
        er_true, er_match = match * 1 + 1, (match * 1 + 1)[:len(self.er_label)] == self.er_label
        er_accuracy = (er_match * 1).mean()
        itr = self.itr(len(self.eeg_label), accuracy, 4)
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
        self.mindoobj.serstatus, self.mindoobj.streaming, self.received_marker, self.gameover = 0, False, True, True

    def start_pause(self):                                                                                                    
        self.pause = True
        
    def sendmsgtohololens(self, msg):
        self.conn.send(msg.encode())

    def get_ssvepcls(self, eeg_signal):
        return self.clf.get_ssvep_command(eeg_signal)

    def get_er_cls(self, eeg_signal):
        return self.clf_er.get_ssvep_command(eeg_signal)

    def stoptcp(self):
        try: self.mindoobj.stop_streaming()
        except: pass
        try: self.conn.close()
        except: pass
        self.mySocket.close()
        if self.robomaster:
            self.ghost_sock.close()
            self.bci_sock.close()

    def sendmsgtohololens(self, msg):
        self.conn.send(msg.encode())

    def get_ssvepcls(self, eeg_signal):
        return self.clf.get_ssvep_command(eeg_signal)

    def get_er_cls(self, eeg_signal):
        return self.clf_er.get_ssvep_command(eeg_signal)

    def pack_and_send(self, cmd):
        Leo_IP, Tello_IP, Send_IP2, BRIPC_IP, DellPC_IP, Phone_IP = "192.168.0.98", "192.168.0.202", "192.168.0.2", "192.168.0.202", "192.168.0.183", "192.168.0.109"
        MESSAGE = struct.pack('!i', cmd)
        IPs = [self.ROBOT_IP, self.host, Leo_IP, Tello_IP]
        Ports = [self.DEFAULT_PORT, self.DRONE_PORT, self.UNITY_PORT]
        for IP in IPs:
            for Port in Ports:
                if IP not in IPs[:-2] or Port != self.DEFAULT_PORT:
                    self.ghost_sock.sendto(MESSAGE, (IP, Port))        
        self.ghost_sock.sendto(MESSAGE, (Phone_IP, self.UNITY_PORT))

    def send_decision(self, eeg_signal, got):
        peak = False
        if got in ["7", "8"]:
            self.send_ghost_er_cmd(eeg_signal, got)
        elif got in ["1", "2", "3", "4", "5", "6"]:
            self.send_ghost_cmd(eeg_signal, got)

    def send_unpause(self, i):
        self.pause, self.robot_reached = False, True
        print("SSVEP BCI Unpaused")
        print("------------------- Trial [{}] -------------------".format(i+1))
        self.sendmsgtohololens("10")

    def send_ghost_cmd(self, eeg_signal, got):
        cls = self.get_ssvepcls(eeg_signal)
        send_marker.send(str(cls + 1))                          
        print("SSVEP BCI output: ", cls + 1)
        self.eeg_label.append(cls + 1)
        if self.robomaster:
            if self.online == True:
                self.pack_and_send(cls + 1)
            else:
                self.pack_and_send(int(got))
                print("Got HoloLens output: ", got)
        self.sendmsgtohololens(str(cls + 1))

    def send_ghost_er_cmd(self, eeg_signal, got):
        peak = False
        cls = 1                       
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
            b_data, addr = self.bci_sock.recvfrom(1024)
            data = struct.unpack('!i', b_data)[0]
            print("Received Feedback from Drone: %s" % data)
            self.robot_reached=True
        self.sendmsgtohololens(str(cls + 1))

    def itr(self,n, p, t):
        itr = 0
        if (p < 0 or 1 < p):
            print('Accuracy need to be between 0 and 1.')
        elif (p < 1 / n):
            itr = 0
            print('ITR might be incorrect because accuracy < chance')
        elif (p == 1):
            itr = np.log2(n) * 60 / t
        else:
            itr = (np.log2(n) + p * np.log2(p) + (1 - p) * np.log2((1 - p) / (n - 1))) * 60 / t
        return itr