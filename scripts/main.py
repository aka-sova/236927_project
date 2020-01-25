import sys
import os

cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))
sys.path.append(os.path.join(cur_loc, 'scripts'))

from udpclient import RClient
from R_func import R_Client_Extend, Target, init_logger
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
    planner_logger_location = os.path.join(cur_loc, 'artifacts','planner_logger.log')

    map_output_loc = os.path.join(cur_loc, 'output','map.p')
    map_output_temp_loc = os.path.join(cur_loc, 'output','map_temp.p')

    map_inflated_output_loc = os.path.join(cur_loc, 'output','map_inflated.p')
    map_inflated_output_temp_loc = os.path.join(cur_loc, 'output','map_inflated_temp.p')

    artifacts_loc = os.path.join(cur_loc, 'artifacts')


    logger = init_logger(logger_location)

    # angles = [angle * math.pi/180 for angle in [-45, 0, 45]]
    angles = [45, 0, -45]

    r = R_Client_Extend(host = "192.168.1.158", 
                        port = 2777,
                        angles = angles,
                        calib_folder = calib_folder,
                        logger = logger,
                        map_output_loc = map_output_loc,
                        map_output_temp_loc = map_output_temp_loc,
                        map_inflated_output_loc = map_inflated_output_loc,
                        map_inflated_output_temp_loc = map_inflated_output_temp_loc,
                        artifacts_loc = artifacts_loc)

    # r.map.save_clear_output()

    try:
        r.connect()
        
        # perform the calibration
        if calibration:
            r.self_calib_pos(range_cmd = (350, 1000), step = 50)
            r.self_calib_rot(range_cmd = (350, 1000), step = 50, direction='right')
        else:

            r.init_sense_thread()           # get info from the sensors
            r.init_mapping_thread()         # update & save map
            r.init_local_sockets()          # send the Pose to the visualizer

            
            destination_location = [100, 0]

            target = Target(target_type = "POS", target_vals = destination_location)
            r.reach_destination(target = target)

            # while True:
            #     path_clear = r.check_line_collision_cross(r.cur_loc, [100,0])
            #     r.logger.info("Path Clear: {}".format(path_clear))
            #     time.sleep(1.0)

            # target = Target(target_type = "POS", target_vals = [0, -200])
            # r.goto(target)
            # target = Target(target_type = "POS", target_vals = [100, -100])
            # r.goto(target)
            # target = Target(target_type = "POS", target_vals = [100 ,0])
            # r.goto(target)
            # target = Target(target_type = "POS", target_vals = [0 ,-200])
            # r.goto(target)
            # target = Target(target_type = "POS", target_vals = [0 ,0])
            # r.goto(target)

            # r.map.save_clear_output()

            # while True:
            #     r.get_data()
            #     print("LOC : X [{}] Y [{}]  ANGLE [{}]".format(r.cur_loc[0], r.cur_loc[1], r.cur_angle))
            #     time.sleep(0.3)


            # debug the RRTtree

            # r.map.create_obstacles()
            # r.map.save_maps_debug()

            # targets_path = r.planner.find_path(r.cur_loc, dest_loc = [50, 450], map = r.map)
            # print("hello")
            


    finally:
        r.terminate()



 



if __name__ == '__main__':
    main(calibration = False)