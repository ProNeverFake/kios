from openai import OpenAI
from keyboard_record import record_voice

# todo a class

def record_and_transcribe(lang = 'en') -> str:
    # Record voice
    record_voice()
    
    client = OpenAI()

    audio_file= open("./recording.mp3", "rb")
    transcription = client.audio.transcriptions.create(
        model="whisper-1", 
        file=audio_file,
        language=lang
    )

    print(transcription.text)

    return transcription.text