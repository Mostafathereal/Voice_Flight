import time 
from utils.mavlink_utils import (
    connect, arm, disarm, set_mode,
    takeoff, land, get_gps, move_relative
)

import utils.utils as utils
import utils.audio_utils as audio_utils

mav_config_file_path = "../config.yaml"
config_data = utils.load_config(mav_config_file_path)

if __name__ == "__main__":

    mav_connection = connect(port=config_data["mav_connect_port"], 
                             baud=config_data["mav_connect_baud_rate"])
    
    set_mode(mav_connection, mode_name="LOITER")

    arm(mav_connection)
    start = time.time()
    duration = 30

    while time.time() - start < duration:
        text = audio_utils.recognize_speech()
        cmd = audio_utils.match_command(text)
        print(cmd)


    print("FAILED")

    disarm(mav_connection)
