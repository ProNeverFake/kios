import sounddevice as sd
import numpy as np
from pynput import keyboard
from pydub import AudioSegment
from scipy.signal import butter, lfilter

'''
record the voice with a high-pass filter with a cut-off frequency of 40 Hz
'''


def high_pass_filter(data: np.ndarray, cutoff, fs, order=5) -> np.ndarray:
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='highpass', analog=False)
    y = lfilter(b, a, data)
    return y

def low_pass_filter(data, cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='lowpass', analog=False)
    y = lfilter(b, a, data)
    return y

def record_voice() -> np.int16:
    '''
    Record voice from the microphone and apply a high-pass filter with a cut-off frequency of 40 Hz.
    copilot generated code.
    '''

    # * Sampling rate. whisper is trained with 16kHz so this is fixed
    fs = 16000
    # Buffer to store the recording
    recording = []
    is_recording = False

    def callback(indata, frames, time, status):
        if is_recording:
            recording.append(indata.copy())

    def on_press(key):
        nonlocal is_recording
        try:
            if key.char == 'v':
                if not is_recording:
                    print("Recording... Press 'v' again to stop.")
                    is_recording = True
                    stream.start()
                else:
                    print("Recording stopped.")
                    is_recording = False
                    stream.stop()
                    return False  # Stop listener
        except AttributeError:
            pass

    # Start recording
    stream = sd.InputStream(callback=callback, channels=1, samplerate=fs)

    # Collect events until released
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

    # Convert list of numpy arrays to a single numpy array
    recording = np.concatenate(recording, axis=0)

    # Apply  filter
    filtered_recording = high_pass_filter(recording, cutoff=40, fs=fs)
    # filtered_recording = low_pass_filter(recording, cutoff=400, fs=fs)

    # Convert numpy array to 16-bit PCM format
    filtered_recording = np.int16(filtered_recording * 32767)

    # Convert numpy array to audio segment
    audio_segment = AudioSegment(
        filtered_recording.tobytes(), 
        frame_rate=fs,
        sample_width=filtered_recording.dtype.itemsize, 
        channels=1
    )

    # Save as wav
    audio_segment.export("recording.mp3", format="mp3")
    print("Recording saved as recording.mp3")

    return filtered_recording


if __name__ == "__main__":
    audio_data = record_voice()
    print("Recording length:", len(audio_data))
    print(audio_data)