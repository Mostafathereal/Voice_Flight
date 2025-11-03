import time 
import utils.mavlink_utils as mavlink_utils
import utils.audio_utils as audio_utils
import utils.utils as utils

mav_config_file_path = "../config.yaml"
config_data = utils.load_config(mav_config_file_path)

if __name__ == "__main__":

    mav_connection = mavlink_utils.connect(port=config_data["mav_connect_port"], 
                             baud=config_data["mav_connect_baud_rate"])
    
    mavlink_utils.set_mode(mav_connection, mode_name="GUIDED")

    # arm(mav_connection)
    start = time.time()
    duration = 30

    while time.time() - start < duration:
        text = audio_utils.recognize_speech()
        cmd = audio_utils.match_command(text)
        print(cmd)
        mavlink_utils.handle_command(mav_connection, cmd)

    print("FAILED")

    # disarm(mav_connection)
