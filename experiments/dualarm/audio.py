from openai import OpenAI
from keyboard_record import record_voice

# todo a class

def record_and_transcribe() -> str:
    # Record voice
    record_voice()
    
    client = OpenAI()

    audio_file= open("./recording.mp3", "rb")
    transcription = client.audio.transcriptions.create(
        model="whisper-1", 
        file=audio_file,
        language="cmn-CN"
    )

    print(transcription.text)

    return transcription.text