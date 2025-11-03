import whisper
import sounddevice as sd
import numpy as np
from rapidfuzz import fuzz
from drone_commands import Command

# -------------------------------
# SPEECH RECOGNITION (Whisper)
# -------------------------------

 # "tiny", "base", "small", "medium", "large"
model = whisper.load_model("base")  # use "tiny" for faster response

def recognize_speech(duration=3, samplerate=16000):
    """Record short audio and transcribe it to text using Whisper"""
    print("🎤 Listening for command...")
    audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype=np.float32)
    sd.wait()

    # Whisper expects float32 numpy array (range -1..1)
    audio = np.squeeze(audio)
    result = model.transcribe(audio, fp16=False)
    text = result['text'].lower().strip()
    if text:
        print(f"🗣️ Heard: “{text}”")
    return text


# -------------------------------
# COMMAND MATCHING
# -------------------------------

def match_command(text):
    """Fuzzy match recognized text to a known Command enum"""
    if not text:
        return None

    best_match = None
    best_score = 0

    for cmd in Command:
        score = fuzz.ratio(text, cmd.value)
        if score > best_score:
            best_match = cmd
            best_score = score

    if best_score > 70:
        print(f"✅ Matched: {best_match.name} ({best_score:.1f}%)")
        return best_match
    else:
        print("❓ No command recognized.")
        return None

# # -------------------------------
# # MAVLINK CONNECTION & CONTROL
# # -------------------------------

# def connect_mavlink(connection_str="udp:127.0.0.1:14550"):
#     print(f"🔌 Connecting to MAVLink at {connection_str} ...")
#     master = mavutil.mavlink_connection(connection_str)
#     master.wait_heartbeat()
#     print(f"✅ Connected to system (system {master.target_system}, component {master.target_component})")
#     return master


# def send_ned_velocity(master, vx, vy, vz, duration=1):
#     """
#     Send velocity command in NED frame (m/s) for a given duration.
#     vx: forward (+x), vy: right (+y), vz: down (+z)
#     """
#     msg = master.mav.set_position_target_local_ned_encode(
#         0, master.target_system, master.target_component,
#         mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#         0b0000111111000111,  # velocity only
#         0, 0, 0,
#         vx, vy, vz,
#         0, 0, 0, 0, 0)
    
#     for _ in range(duration):
#         master.mav.send(msg)
#         time.sleep(1)





# # -------------------------------
# # MAIN LOOP
# # -------------------------------

# if __name__ == "__main__":
#     # master = connect_mavlink("udp:127.0.0.1:14550")

#     while True:
#         text = recognize_speech()
#         cmd = match_command(text)
#         print(cmd)
#         # if cmd:
#             # handle_command(master, cmd)
#         # if cmd == Command.LAND:
#         #     break