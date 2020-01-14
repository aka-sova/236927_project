import sys
import os

cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))
sys.path.append(os.path.join(cur_loc, 'scripts'))

from udpclient import RClient
from R_func import R_Client_Extend, Target
import time
import math

import logging

global logger


### Basic usage:


# r=RClient("192.168.1.151",2777)
# r.drive(480,480)
# print(r.sense())

# time.sleep(2)
# r.terminate()


def main(calibration: bool):




    # create output/artifacts dirs
    os.makedirs(os.path.join(cur_loc, 'artifacts'), exist_ok=True)
    os.makedirs(os.path.join(cur_loc, 'output'), exist_ok=True)

    
    calib_folder = os.path.join(cur_loc, 'scripts', 'calib')
    logger_location = os.path.join(cur_loc, 'artifacts','logger.log')
    map_output_loc = os.path.join(cur_loc, 'output','map.csv')
    map_output_temp_loc = os.path.join(cur_loc, 'output','map_temp.csv')



    # angles = [angle * math.pi/180 for angle in [-45, 0, 45]]
    angles = [45, 0, -45]
    final_loc = [1000, 1000] # X, Y of the final location

    r = R_Client_Extend(host = "192.168.1.158", 
                        port = 2777,
                        angles = angles,
                        calib_folder = calib_folder,
                        logger_location = logger_location,
                        map_output_loc = map_output_loc,
                        map_output_temp_loc = map_output_temp_loc)

    # r.map.save_clear_output()

    try:
        r.connect()
        
        # perform the calibration
        if calibration:
            r.self_calib_pos(range_cmd = (350, 1000), step = 50)
            r.self_calib_rot(range_cmd = (350, 1000), step = 50, direction='right')
        else:

            r.init_sense_thread()
            r.init_mapping_thread()

            # initialize the sockets, which will send the Location to the visualizer
            r.init_local_sockets(pose_socket = True)

            

            target = Target(target_type = "POS", target_vals = [0, -200])
            r.goto(target)
            target = Target(target_type = "POS", target_vals = [100, -100])
            r.goto(target)
            target = Target(target_type = "POS", target_vals = [100 ,0])
            r.goto(target)
            target = Target(target_type = "POS", target_vals = [0 ,-200])
            r.goto(target)
            target = Target(target_type = "POS", target_vals = [0 ,0])
            r.goto(target)

            # r.map.save_clear_output()

            # while True:
            #     r.get_data()
            #     print("LOC : X [{}] Y [{}]  ANGLE [{}]".format(r.cur_loc[0], r.cur_loc[1], r.cur_angle))
            #     time.sleep(0.3)
            


    finally:
        pass
        # r.terminate()



 



if __name__ == '__main__':
    main(calibration = False)