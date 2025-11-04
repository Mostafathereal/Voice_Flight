import time 
import utils.mavlink_utils as mavlink_utils
import utils.audio_utils as audio_utils
import utils.utils as utils
from pymavlink import mavutil

mav_config_file_path = "../config.yaml"
config_data = utils.load_config(mav_config_file_path)

if __name__ == "__main__":

    mav_connection = mavlink_utils.connect(port=config_data["mav_connect_port"], 
                             baud=config_data["mav_connect_baud_rate"])
    
    mavlink_utils.set_mode(mav_connection, mode_name="GUIDED")

    # # arm(mav_connection)
    # start = time.time()
    # duration = 120

    # mavlink_utils.set_mode(mav_connection, "GUIDED")
    # time.sleep(3)
    # mavlink_utils.arm(mav_connection)
    # time.sleep(3)




    # mavlink_utils.takeoff(mav_connection, altitude=5.0)
    

    
    # time.sleep(20)
    # mavlink_utils.land(mav_connection)
    # time.sleep(20)
    # mavlink_utils.disarm(mav_connection)

    while True: #time.time() - start < duration:
        text = audio_utils.recognize_speech()
        cmd = audio_utils.match_command(text)
        print(cmd)
        mavlink_utils.handle_command(mav_connection, cmd)

    # print("FAILED")

    # disarm(mav_connection)
