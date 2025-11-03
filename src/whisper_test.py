

# import time
# import whisper
# import sounddevice as sd
# import numpy as np
# from enum import Enum
# from rapidfuzz import fuzz
# from pymavlink import mavutil


# # -------------------------------
# # ENUM DEFINITIONS
# # -------------------------------

# class Command(Enum):
#     ARM = "hey drone arm"
#     DISARM = "hey drone disarm"
#     TAKE_OFF = "hey drone take off"
#     LAND = "hey drone land"
#     HOVER = "hey drone hover"
#     MOVE_FORWARD = "hey drone move forward"
#     MOVE_BACKWARD = "hey drone move backward"
#     MOVE_LEFT = "hey drone move left"
#     MOVE_RIGHT = "hey drone move right"



# # -------------------------------
# # SPEECH RECOGNITION (Whisper)
# # -------------------------------

#  # "tiny", "base", "small", "medium", "large"
# model = whisper.load_model("small")  # use "tiny" for faster response

# def listen_and_transcribe(duration=3, samplerate=16000):
#     """Record short audio and transcribe it to text using Whisper"""
#     print("🎤 Listening for command...")
#     audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype=np.float32)
#     sd.wait()

#     # Whisper expects float32 numpy array (range -1..1)
#     audio = np.squeeze(audio)
#     result = model.transcribe(audio, fp16=False)
#     text = result['text'].lower().strip()
#     if text:
#         print(f"🗣️ Heard: “{text}”")
#     return text


# # -------------------------------
# # COMMAND MATCHING
# # -------------------------------

# def match_command(text):
#     """Fuzzy match recognized text to a known Command enum"""
#     if not text:
#         return None

#     best_match = None
#     best_score = 0

#     for cmd in Command:
#         score = fuzz.ratio(text, cmd.value)
#         if score > best_score:
#             best_match = cmd
#             best_score = score

#     if best_score > 70:
#         print(f"✅ Matched: {best_match.name} ({best_score:.1f}%)")
#         return best_match
#     else:
#         print("❓ No command recognized.")
#         return None


# if __name__ == "__main__":

#     print("🎧 Voice command control ready. Say a command like:")
#     print("  'take off', 'move forward', 'hover', 'land'")

#     while True:
#         text = listen_and_transcribe(duration=3)
#         cmd = match_command(text)
#         print(cmd)