import matplotlib.pyplot as plt
from scipy.io import wavfile
import wavio
import os
from pydub import AudioSegment
import numpy as np
from keras.utils import np_utils
import pickle
from datetime import datetime
import wave
#import librosa
import soundfile as sf

# Calculate and plot spectrogram for a wav audio file
def graph_spectrogram(wav_file):
    rate, data = get_wav_info(wav_file)
    nfft = 200 # Length of each window segment
    fs = 8000 # Sampling frequencies
    noverlap = 120 # Overlap between windows
    nchannels = data.ndim
    if nchannels == 1:
        pxx, freqs, bins, im = plt.specgram(data, nfft, fs, noverlap = noverlap)
    elif nchannels == 2:
        pxx, freqs, bins, im = plt.specgram(data[:,0], nfft, fs, noverlap = noverlap)
    return pxx


# Load a wav file
def get_wav_info(wav_file):
    #rate, data = wavfile.read(wav_file)
    wav = wavio.read(wav_file)
    rate, data = wav.rate, wav.data
    return rate, data


# Used to standardize volume of audio clip
def match_target_amplitude(sound, target_dBFS):
    change_in_dBFS = target_dBFS - sound.dBFS
    return sound.apply_gain(change_in_dBFS)


# Load raw audio files for speech synthesis
def load_raw_audio_old():
    activates = []
    backgrounds = []
    negatives = []
    for filename in os.listdir("./raw_data/activates"):
        if filename.endswith("wav"):
            activate = AudioSegment.from_wav("./raw_data/activates/"+filename)
            activates.append(activate)
    for filename in os.listdir("./raw_data/backgrounds"):
        if filename.endswith("wav"):
            background = AudioSegment.from_wav("./raw_data/backgrounds/"+filename)
            backgrounds.append(background)
    for filename in os.listdir("./raw_data/negatives"):
        if filename.endswith("wav"):
            negative = AudioSegment.from_wav("./raw_data/negatives/"+filename)
            negatives.append(negative)
    return activates, negatives, backgrounds


# my version of load_raw_audio
def load_raw_audio(clips_location_dict = {'enough': 'recorded_clips/enoughs',
                                          'backgrounds': 'recorded_clips/fullbackgrounds_trimmed_5sec',
                                          'not_enough':  'recorded_clips/notenoughs',
                                          'empty': 'recorded_clips/empties'}):
    enoughs = []
    fullbackgrounds = []
    notenoughs = []
    empties = []
    clips_dict = {}
    for k in clips_location_dict.keys():
        clips_dict[k] = []
    for k, v in clips_location_dict.items():
        print('{:.<15}...'.format(k), end='')
        for filename in os.listdir(v):
            if filename.endswith('wav'):
#                print(filename)
                clip = AudioSegment.from_wav('{}/{}'.format(v, filename))
                clips_dict[k].append(clip)
        print('{} clips.'.format(len(clips_dict[k])))
#    print('enoughs...')
#    for filename in os.listdir(enoughs_folder):
#        if filename.endswith("wav"):
#            print(filename)
#            enough = AudioSegment.from_wav("{}/{}".format(enoughs_folder, filename))
#            enoughs.append(enough)
#    print('backgrounds...')
#    for filename in os.listdir("recorded_clips/fullbackgrounds_trimmed_5sec"):
#        if filename.endswith("wav"):
#            print(filename)
#            fullbackground = AudioSegment.from_wav("recorded_clips/fullbackgrounds_trimmed_5sec/"+filename)
#            fullbackgrounds.append(fullbackground)
#    print('notenoughs...')
#    for filename in os.listdir("recorded_clips/notenoughs"):
#        if filename.endswith("wav"):
#            print(filename)
#            notenough = AudioSegment.from_wav("recorded_clips/notenoughs/"+filename)
#            notenoughs.append(notenough)
#    print('empties...')
#    for filename in os.listdir("recorded_clips/empties"):
#        if filename.endswith("wav"):
#            print(filename)
#            empty = AudioSegment.from_wav("recorded_clips/empties/"+filename)
#            empties.append(empty)
    return clips_dict


# Preprocess the audio to the correct format. trims to 10 sec.
def preprocess_audio(filename):
    # Trim or pad audio segment to 10000ms
    padding = AudioSegment.silent(duration=10000)
    segment = AudioSegment.from_wav(filename)[:10000]
    segment = padding.overlay(segment)
    # Set frame rate to 44100
    segment = segment.set_frame_rate(44100)
    # Export as wav
    segment.export(filename, format='wav')


def split_into_2_5sec_files(folder_name, idx):
    filename = 'recorded_clips/{}/{:0>2}.wav'.format(folder_name, idx)
    print(filename)
    padding1 = AudioSegment.silent(duration=5000)
    segment1 = AudioSegment.from_wav(filename)[:5000]
    segment1 = padding1.overlay(segment1)
    segment1 = segment1.set_frame_rate(44100)
    
    padding2 = AudioSegment.silent(duration=5000)
    segment2 = AudioSegment.from_wav(filename)[5000:]
    segment2 = padding2.overlay(segment2)
    segment2 = segment2.set_frame_rate(44100)
    
    # Export as wav
    segment1.export('recorded_clips/{}/{:0>2}.wav'.format(folder_name, idx), format='wav')
    segment2.export('recorded_clips/{}/{:1>2}.wav'.format(folder_name, idx), format='wav')
    

def trim_to_5sec(folder_name, idx):
    filename = 'recorded_clips/{}/{:0>2}.wav'.format(folder_name, idx)
    print(filename)
    padding1 = AudioSegment.silent(duration=5000)
    segment1 = AudioSegment.from_wav(filename)[:5000]
    segment1 = padding1.overlay(segment1)
    segment1 = segment1.set_frame_rate(44100)
        
    # Export as wav
    segment1.export('recorded_clips/{}/{:0>2}.wav'.format(folder_name, idx), format='wav')
    

def get_random_time_segment(segment_ms):
    """
    Gets a random time segment of duration segment_ms in a 5,000 ms audio clip.
    
    Arguments:
    segment_ms -- the duration of the audio clip in ms ("ms" stands for "milliseconds")
    
    Returns:
    segment_time -- a tuple of (segment_start, segment_end) in ms
    """
    segment_start = np.random.randint(low=0, high=5000-segment_ms)
    segment_end = segment_start + segment_ms - 1
    return (segment_start, segment_end)


def is_overlapping(segment_time, previous_segments):
    """
    Checks if the time of a segment overlaps with the times of existing segments.
    
    Arguments:
    segment_time -- a tuple of (segment_start, segment_end) for the new segment
    previous_segments -- a list of tuples of (segment_start, segment_end) for the existing segments
    
    Returns:
    True if the time segment overlaps with any of the existing segments, False otherwise
    """
    segment_start, segment_end = segment_time
    overlap = False

    for previous_start, previous_end in previous_segments:
        if segment_start <= previous_end and segment_end >= previous_start:
            overlap = True
    return overlap


def insert_audio_clip(background, audio_clip, previous_segments):
    """
    Insert a new audio segment over the background noise at a random time step, ensuring that the 
    audio segment does not overlap with existing segments.
    
    Arguments:
    background -- a 10 second background audio recording.  
    audio_clip -- the audio clip to be inserted/overlaid. 
    previous_segments -- times where audio segments have already been placed
    
    Returns:
    new_background -- the updated background audio
    """
    segment_ms = len(audio_clip)
    segment_time = get_random_time_segment(segment_ms)

    while is_overlapping(segment_time, previous_segments) == True:
        segment_time = get_random_time_segment(segment_ms)
    previous_segments.append(segment_time)

    new_background = background.overlay(audio_clip, position = segment_time[0])
    return new_background, segment_time


def make_training_sample(backgrounds, samples_to_add, label):
    """
    generate a training sample, in the form of a numpy array from the spectrogram, 
    and a multiclass label
    
    arguments:
    backgrounds -- list of long background samples, from load_raw_audio
    samples_to_add -- list of samples to add, one of: enoughs, notenoughs, empties.  
    from load_raw_audio
    label -- string label to assign this sample, one of: 'enough', 'not_enough', 'empty'
    
    returns:
    x -- numpy array of training sample spectrogram
    y -- label
    """

    sample_idx = np.random.randint(len(samples_to_add))
    bkgnd_idx = np.random.randint(len(backgrounds))

    sample_to_add = samples_to_add[sample_idx]
    background = backgrounds[bkgnd_idx]
    
    # make background quieter
    background = background - 20

    previous_segments = []

    if (label=='empty') and (np.random.normal()>0):
        # if label is empty, 50% chance we don't put in an empty clip
        background = background
        sample_idx = -1
    else:
        background, segment_time = insert_audio_clip(background, sample_to_add, previous_segments)

    y = label

    background = match_target_amplitude(background, -20.0)

    file_handle = background.export("train" + ".wav", format="wav")

    x = graph_spectrogram("train.wav")
    return x, y, sample_idx, bkgnd_idx

# make a set of training samples with the same label
def make_training_samples(backgrounds, samples_to_add, label, n_samples, Tx, n_freq, Ty):
    if label not in ['enough', 'not_enough', 'empty']:
        print('Label must be one of: enough, not_enough, empty.')
        return 0
    
    X = np.zeros((n_samples, Tx, n_freq))
    Y = []
    sample_indices = np.zeros(n_samples)
    bkgnd_indices = np.zeros(n_samples)
    print('sample: ', end='')
    for i in range(n_samples):
        print(' {}'.format(i), end='')
        x, y, sample_idx, bkgnd_idx = make_training_sample(backgrounds, samples_to_add, label)
        X[i] = x.transpose()
        Y.append(y)
        sample_indices[i] = sample_idx
        bkgnd_indices[i] = bkgnd_idx
    plt.show()
    plt.hist(sample_indices)
    plt.title('sample index distribution')
    plt.show()

    plt.hist(bkgnd_indices)
    plt.title('background index distribution')
    plt.show()
    return X, Y

def make_features_and_labels(n_enoughs, n_notenoughs, n_empties,
                             enoughs, notenoughs, empties, backgrounds, Tx, n_freq, Ty):
    X_en, y_en = make_training_samples(backgrounds, enoughs, 'enough', n_enoughs, Tx, n_freq, Ty)
    X_nen, y_nen  = make_training_samples(backgrounds, notenoughs, 'not_enough',
                                          n_notenoughs, Tx, n_freq, Ty) 
    X_emp, y_emp  = make_training_samples(backgrounds, empties, 'empty', n_empties, Tx, n_freq, Ty)

    # combine into one feature dataset and label dataset
    X = np.concatenate([X_en, X_nen, X_emp])
    y = np.concatenate([y_en, y_nen, y_emp])

    print('size of X:  {}'.format(X.shape))
    print('length of y: {}'.format(len(y)))
    return X, y

# save to disk, with optional file comment
def save_datasets_to_disk(X, y, file_folder, file_comment):

    file = open('{}/X_{}.pkl'.format(file_folder, file_comment), 'ab') 
    pickle.dump(X, file)
    print('Features saved to: {}/X_{}.pkl'.format(file_folder, file_comment))
    file.close() 

    file = open('{}/y_{}.pkl'.format(file_folder, file_comment), 'ab') 
    pickle.dump(y, file)                      
    print('Labels saved to: {}/y_{}.pkl'.format(file_folder, file_comment))
    file.close() 


def encode_and_oh_labels(labels):
    label_encoder_dict = {'enough': 1, 'not_enough': 2, 'empty': 0}
    y_encoded = np.array([label_encoder_dict[l] for l in labels])
    return np_utils.to_categorical(y_encoded)

# pads/trims runtime clip to 5 seconds, and saves as runtime_temp.wav. option for background file.
def preprocess_runtime_clip(filename, background_filepath=None):
    if background_filepath:
        padding = AudioSegment.from_wav(background_filepath)[:5000]
    else:
        padding = padding = AudioSegment.silent(duration=5000)
    segment = AudioSegment.from_wav(filename)[:5000]
    segment = padding.overlay(segment)
    segment = segment.set_frame_rate(44100)
        
    # Export as wav
    segment.export('runtime_temp.wav', format='wav')

# reads in audio clip and pushes it through model to generate prediction
def run_model_on_clip(model, clip_filename='runtime_temp.wav'):
    label_decoder_dict = {0:'empty', 1:'enough', 2:'not_enough'}
    x = graph_spectrogram(clip_filename)
    x = x.transpose().reshape((1, x.shape[1], x.shape[0]))
    preds = model.predict(x)
    return label_decoder_dict[np.argmax(preds)], preds

def record_and_process_5_seconds(idx, samp_rate, chunk, record_secs, stream,chans,
                                 form_1, audio, model):
    #print("recording {}".format(idx))
    print('Clip start: {}...'.format(datetime.now().strftime("%m/%d/%Y %H:%M:%S")), end='')
    frames = []

    # loop through stream and append audio chunks to frame array
    for ii in range(0,int((samp_rate/chunk)*record_secs)):
        data = stream.read(chunk)
        frames.append(data)

    #print("finished recording {}".format(idx))

    # save the audio frames as .wav file
    wavefile = wave.open('runtime_temp.wav','wb')
    wavefile.setnchannels(chans)
    wavefile.setsampwidth(audio.get_sample_size(form_1))
    wavefile.setframerate(samp_rate)
    wavefile.writeframes(b''.join(frames))
    wavefile.close()

    #print('finished saving {}'.format(idx))

    preprocess_runtime_clip('runtime_temp.wav')

    # feed current clip to model and get prediction
    pred, preds = run_model_on_clip(model, 'runtime_temp.wav')
    preds_nice = ', '.join(['{:.4f}'.format(p) for p in preds[0]])
    print('   prediction: {}.   [{}]'.format(pred, preds_nice))
    return pred


# augment data by using librosa.effects library to pitch shift and time stretch
def data_augmentation(folder_path, n_pitch_shifts, n_time_stretches, ps_sigma=1.5, ts_sigma=0.2):
    import librosa
    for filename in os.listdir(folder_path):
        if filename.endswith('wav') and len(filename) < 8:
            audio_path = folder_path + filename

            # load base audio clip
            base_audio, sampling_rate = librosa.load(audio_path, sr=44100)

            # randomly collect some pitch shift values
            if n_pitch_shifts == 0:
                pitch_shifts = [0]
            else:
                pitch_shifts = np.random.normal(0, ps_sigma/2, size = n_pitch_shifts)  #1.5 

            # randomly collect some time stretch values
            if n_time_stretches == 0:
                time_stretches = [1]
            else:
                time_stretches = np.random.normal(1, ts_sigma/2, size = n_time_stretches)  #0.2

            print(filename)
            print('Pitch shifts (half steps):   {}'.format(pitch_shifts))
            print('Time stretches (multiplier): {}'.format(time_stretches))

            # choose a pitch shift value and time stretch value (really in a loop)
            for i, ps in enumerate(pitch_shifts):
                for j, ts in enumerate(time_stretches):
#                    print('Pitch shift (half steps): {:.3f}.  Time stretch (multiplier): {:.3f}...'.format(ps, ts), 
#                          end='')

                    temp_audio = librosa.effects.pitch_shift(base_audio, sampling_rate, ps)
                    new_audio = librosa.effects.time_stretch(temp_audio, ts)

                    filename_minus_wav = filename[:-4]
                    new_audio_filename = '{}_{:0>2}_{:0>2}.wav'.format(filename_minus_wav, i, j)
                    sf.write(folder_path+new_audio_filename, new_audio, sampling_rate)
            
