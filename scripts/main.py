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


def main():




    # create output/artifacts dirs
    os.makedirs(os.path.join(cur_loc, 'artifacts'), exist_ok=True)
    os.makedirs(os.path.join(cur_loc, 'output'), exist_ok=True)

    
    calib_folder = os.path.join(cur_loc, 'scripts', 'calib')
    logger_location = os.path.join(cur_loc, 'artifacts','logger.log')




    angles = [angle * math.pi/180 for angle in [-45, 0, 45]]
    final_loc = [1000, 1000] # X, Y of the final location

    r = R_Client_Extend(host = "192.168.1.158", 
                        port = 2777,
                        angles = angles,
                        calib_folder = calib_folder,
                        logger_location = logger_location)

    # print(int(r.pos_interp(abs(20))))

    # while(True):
    #     r.turn(90)
    #     time.sleep(3.0)
    #     r.drive_distance(20)
    #     time.sleep(3.0)

    try:

        r.connect()
        
        # perform the calibration
        r.self_calib_pos(range_cmd = (350, 1000), step = 50)

        r.self_calib_rot(range_cmd = (350, 1000), step = 50, direction='right')

    finally:
        r.terminate()


    # dummy_target = Target(target_type = "POS", target_vals = [100, 100])
    # r.goto(dummy_target)
 



if __name__ == '__main__':
    main()