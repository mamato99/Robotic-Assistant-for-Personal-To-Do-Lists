#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String, Float32MultiArray
import numpy as np
import pickle
import os

from identification.deep_speaker.audio import get_mfcc
from identification.deep_speaker.model import get_deep_speaker
from identification.utils import batch_cosine_similarity, dist2id

from ros_audio_pkg.srv import *
from config import *

class SpeakerReIdentification():

    def __init__(self):
        # Load model
        self.model = get_deep_speaker(os.path.join(REF_PATH,'deep_speaker.h5'))
        
        # Load existring embedding if exists
        self.data = self._load_data()
        self.pred_identity = None
        self.current_user = None
        self.queue = []        
        
    def _load_data(self):
        try:
            with open(os.path.join(REF_PATH, EMBEDDING_FILENAME), 'rb') as fh:
                data = pickle.load(fh)
            
            print('[RE-IDENTIFICATION] Embedding loaded:')
            for u in set(data['y']):
                print('  >',u,data['y'].count(u))
        except Exception as e:
            data = dict()
            data['X'] = list()
            data['y'] = list()
        return data
    
    def _save_data(self):
        try:
            with open(os.path.join(REF_PATH, EMBEDDING_FILENAME), 'wb') as fh:
                pickle.dump(self.data, fh)
        except:
            print("[RE-IDENTIFICATION] Can't save the state.")
        
    def start(self):
        rospy.init_node('speaker_reidentification_node')
        rospy.Service('identify', Identify, self._identify)
        rospy.Service('save_id', SaveIdentity, self._save_id)
        rospy.spin()
        
    def _get_embedding(self, audio:Int16MultiArray):
        audio_data = np.array(audio.data)

        # to float32
        audio_data = audio_data.astype(np.float32, order='C') / 32768.0
        
        # Processing
        ukn = get_mfcc(audio_data, RATE)
        
        # Prediction
        emb = self.model.predict(np.expand_dims(ukn, 0)) # to float32
        
        return emb
        
    def _identify(self, req:IdentifyRequest):        
        
        emb = self._get_embedding(req.audio)
        
        if len(self.data['X']) > 0:
            # Distance between the sample and the support set
            emb_voice = np.repeat(emb, len(self.data['X']), 0)
            cos_dist = batch_cosine_similarity(np.array(self.data['X']), emb_voice)
            # Matching
            self.pred_identity = dist2id(cos_dist, self.data['y'], TH, mode='avg')          
        
        print('[RE-IDENTIFICATION] Predicted:',self.pred_identity)
 
        emb_to_return = Float32MultiArray()
        emb_to_return.data = emb[0]
        return IdentifyResponse(self.pred_identity, emb_to_return)
    
    def _save_id(self, req:SaveIdentityRequest):
        audio = req.audio
        emb = np.array(req.emb.data)
        id = req.id

        if audio is not None and emb.shape == (0,):
            emb = self._get_embedding(audio)
            emb = emb[0]
               
        if self.data['y'].count(id) < MAX_EMBEDDING:
            self.data['X'].append(emb)
            self.data['y'].append(id)
            self._save_data()
            print('[RE-IDENTIFICATION] Saving {}'.format(id))
        else:
            print('[RE-IDENTIFICATION] Almost {} embedding for {}'.format(MAX_EMBEDDING, id))
        
        return SaveIdentityResponse('[ACK]')

if __name__ == '__main__':
    try:
        identifcator = SpeakerReIdentification()
        print('[RE-IDENTIFICATION] Start')
        identifcator.start()
    except rospy.ROSInterruptException:
        pass