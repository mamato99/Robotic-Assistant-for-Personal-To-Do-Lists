#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np

import speech_recognition as sr

from ros_audio_pkg.srv import ListenStart, ListenStartResponse, ListenStop, ListenStopResponse
from config import *

class VoiceDetection():
    
    def __init__(self, verbose = 0):
        # Initialize a Recognizer
        self.r = sr.Recognizer()
        
        # Audio source
        self.m = sr.Microphone(device_index=MIC_INDEX,
                    sample_rate=SAMPLE_RATE,
                    chunk_size=CHUNK_SIZE)
        
        self.verbose = verbose
        
        print("[VOICE DETECTION] init done")
        
    def _callback(self, recognizer, audio):
        data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
        data_to_send = Int16MultiArray()
        data_to_send.data = data

        self.pub.publish(data_to_send)
    
    def start(self, normaly_open=True):
        rospy.init_node('voice_detection_node', anonymous=True)
        self.pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)        
        
        rospy.Service('listen_start', ListenStart, self.listen_start)
        rospy.Service('listen_stop', ListenStop, self.liset_stop)
        
        # Calibration within the environment
        # we only need to calibrate once, before we start listening
        print("[VOICE DETECTION] Calibrating...")
        with self.m as source:
            self.r.adjust_for_ambient_noise(source,duration=NOISE_DURATION)
        print("[VOICE DETECTION] Calibration finished")
        
        if normaly_open:
            self.listen_start()
        
        rospy.spin()
        
    def listen_start(self, data=None):
        # start listening in the background
        # `stop_listening` is now a function that, when called, stops background listening
        self.stop_listening = self.r.listen_in_background(self.m, self._callback)
        print("[VOICE DETECTION] Recording...")
        return ListenStartResponse('[ACK]')
        
    def liset_stop(self, data=None):
        self.stop_listening()
        print("[VOICE DETECTION] Recording stopped")
        return ListenStopResponse('[ACK]')
        
if __name__ == "__main__":
    try:
        node = VoiceDetection()
        node.start() # Mic activate at the beginning as default
    except rospy.ROSInterruptException:
        pass
        

