#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import Int16MultiArray, String

from pepper_nodes.srv import Text2Speech, Text2SpeechRequest, Text2SpeechResponse
from ros_audio_pkg.srv import *

import os, time
from gtts import gTTS
from playsound import playsound

from config import * 
from reminder import Reminder

reidenntification_script = ["Again \'Hi Pepper\'", "Please, again \'Hi Pepper\'", "Ok, now please say \'Hello Pepper\'","Another time \'Hello Pepper\'","Just another \'Hello Pepper\'","In the end, please say \'Hello\'"]

class T2SInterface():
    
    def __init__(self):
        if PEPPER:
            rospy.wait_for_service('/tts')
            self._tts = rospy.ServiceProxy("/tts", Text2Speech)
        rospy.wait_for_service('listen_start')
        self._mic_on = rospy.ServiceProxy('listen_start', ListenStart)    
        rospy.wait_for_service('listen_stop')
        self._mic_off = rospy.ServiceProxy('listen_stop', ListenStop)

    def speech(self, text: str):
        self._mic_off()
        if PEPPER:
            msg = Text2SpeechRequest()
            msg.speech = text
            resp = self._tts(text)
            rospy.sleep(len(text)*CHAR_SPEED)
        else:
            try:
                to_speak = gTTS(text=text, lang=LANGUAGE, slow=False)
                to_speak.save("temp.wav")
                playsound("temp.wav")
                os.remove("temp.wav")
            except AssertionError:
                pass
        # time.sleep(1)
        print("[OUT]:",text)
        self._mic_on()
        
class ReIdentificationInterface():
    def __init__(self):
        rospy.wait_for_service('identify')
        self._identify_srv = rospy.ServiceProxy('identify',Identify)
        rospy.wait_for_service('save_id')
        self._save_id_srv = rospy.ServiceProxy('save_id',SaveIdentity)
    
    def identify(self, audio_track):
        req = IdentifyRequest(audio_track)
        resp_identify = self._identify_srv(req)
        return resp_identify.id, resp_identify.emb
    
    def save_id(self, id, emb=None, audio_track=None):
        req = SaveIdentityRequest(audio_track,emb,id)
        rsp = self._save_id_srv(req)

def main():
    rospy.init_node('dialog_interface')
    
    print('[CHATBOT] Waiting for services')
    
    ## Services for conversation with the chatbot
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
    print('[CHATBOT] RASA services ready')
    
    rid = ReIdentificationInterface()
    print('[CHATBOT] Re-identification services ready')
    
    ## Classes for integration
    reminder = Reminder(DB_PATH)    
    t2s = T2SInterface()
    print('[CHATBOT] READY')
    
    while not rospy.is_shutdown():
        print('[CHATBOT] Wait for user')
        audio_track = rospy.wait_for_message("voice_data", Int16MultiArray)
        user,emb = rid.identify(audio_track)
        
        if user != '':
            dialogue_service('Hi')
            bot_answer = dialogue_service(f"I'm {user}")
            t2s.speech(bot_answer.answer)
        else:
            t2s.speech("Hi, I don't know you, what is your name?")
            user = rospy.wait_for_message("voice_txt", String)
            user = user.data.split(' ')[-1]
            rid.save_id(user,emb)
            t2s.speech(f"Hi {user}, repeat \'Hi Pepper\'")
            
            for i in range(MAX_EMBEDDING-1):
                audio_track = rospy.wait_for_message("voice_data", Int16MultiArray)
                rid.save_id(user,audio_track=audio_track)
                t2s.speech(reidenntification_script[i])
            
            dialogue_service('Hi')
            bot_answer = dialogue_service(f"I'm {user}")
            t2s.speech(bot_answer.answer)
        
        print('[CHATBOT] Current user:',user)
        reminder.set_username(user)
        rem = reminder.remind_me()
        if rem is not None:
            t2s.speech(rem)
              
        session = True
        while session:
            message = rospy.wait_for_message("voice_txt", String)
            print('[IN]:',message.data)
            
            try:
                bot_answer = dialogue_service(message.data)
                t2s.speech(bot_answer.answer)
            except rospy.ServiceException as e:
                print("[CHATBOT] Service call failed: %s"%e)
                
            if 'bye' in message.data:
                session = False
                user = None
                print('[CHATBOT] reset')
                
            rem = reminder.remind_me()
            if rem is not None:
                t2s.speech(rem)
                
if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass