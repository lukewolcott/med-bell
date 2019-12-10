#!/usr/bin/env python
# coding: utf-8

# In[1]:


import keras
from pydub import AudioSegment
from mb_utils import *
from datetime import datetime
import pyaudio
import wave
import time
import h5py

print('Loading keras model...')
model = keras.models.load_model('trained_models/4943569.h5')

# choose the 'Built-in Microphone'
dev_index = 2

# audio settings
form_1 = pyaudio.paInt16 # 16-bit resolution
chans = 1 # 1 channel
samp_rate = 44100 # 44.1kHz sampling rate
chunk = 4096 # 2^12 samples for buffer
record_secs = 5 # seconds to record

def get_recording_state():
    file = open('current-recording-state.txt')
    rec_state = file.read().replace('\n', '')
    file.close()
    return rec_state

rec_state = get_recording_state()
print('Recording state: {}'.format(rec_state))

# only turn on recording if triggered by med-bell.py
while rec_state == 'off':
    time.sleep(5)
    rec_state = get_recording_state()

# create pyaudio instantiation
audio = pyaudio.PyAudio() 
print('Starting recording stream...')

# create pyaudio stream
stream = audio.open(format = form_1,rate = samp_rate,channels = chans,                     input_device_index = dev_index,input = True,                     frames_per_buffer=chunk)



# process five seconds at a time and return predictions, until triggered to stop
print('Recording...')
while rec_state == 'on':
    pred = record_and_process_5_seconds(0, samp_rate, chunk, record_secs, stream, chans, 
                                        form_1, audio, model)
    file = open('current-prediction.txt', 'w')
    file.write(pred)
    file.close()
    # print('Wrote prediction to current-prediction.txt')
    rec_state = get_recording_state()

# stop the stream, close it, and terminate the pyaudio instantiation
stream.stop_stream()
stream.close()
print('stream closed.')
audio.terminate()

