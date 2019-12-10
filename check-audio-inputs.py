#import keras
from pydub import AudioSegment
#from mb_utils import *
from datetime import datetime
import pyaudio
import wave

# initialization: load trained model
#model = keras.models.load_model('trained_models/4109505.h5')

# set up to listen to audio
p = pyaudio.PyAudio()
for ii in range(p.get_device_count()):
    print(p.get_device_info_by_index(ii).get('name'))

# shows that USB mic is device 2
