import numpy as np
import threading
import time

class SimulatedEEG(threading.Thread):
    def __init__(self, stream_name, channels=64, srate=500, time=5):
        threading.Thread.__init__(self)
        self.stream_name = stream_name
        self.ch = channels
        self.srate = srate
        self.timepoints = int(srate * time)
        self.eeg_final = np.zeros((channels, self.timepoints))
        self.serstatus = 1
        self.streaming = True

    def run(self):
        print("Simulated EEG streaming began")
        while self.streaming:
            # Generate dummy EEG data
            sample = np.random.rand(self.ch, 30)
            self.shift_eeg(sample)
            time.sleep(0.06)  # Simulate sampling rate

    def shift_eeg(self, eeg_block):
        n_steps = eeg_block.shape[1]
        self.eeg_final[:, :-n_steps] = self.eeg_final[:, n_steps:]
        self.eeg_final[:, -n_steps:] = eeg_block

    def stop_streaming(self):
        self.streaming = False
        print("Simulated EEG streaming stopped")