import os
import pathlib

print(os.getcwd())

PEPPER = True #False

DB_PATH=str(pathlib.Path(__file__).parent.absolute()) + "/src/rasa_ros/database.db"

CHAR_SPEED = 0.01

## Speech to text 
LANGUAGE = 'en-US'

## Microphone
MIC_INDEX = 5 #None
SAMPLE_RATE = 16000
CHUNK_SIZE = 1024
NOISE_DURATION = 3

## Speaker identification variables
REF_PATH = os.path.dirname(os.path.abspath(__file__)) + "/src/ros_audio_pkg/src"
MODEL_PATH = os.path.join(REF_PATH,'deep_speaker.h5')
EMBEDDING_FILENAME = os.path.join(REF_PATH,'embedding.pk')
RATE = 16000
TH = 0.75

MAX_EMBEDDING = 7
