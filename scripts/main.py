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

    global logger

    # create output/artifacts dirs
    os.makedirs(os.path.join(cur_loc, 'artifacts'), exist_ok=True)
    os.makedirs(os.path.join(cur_loc, 'output'), exist_ok=True)

    # create a logger
    logging.basicConfig(filename=os.path.join(cur_loc, 'artifacts','logger.log'), 
                        level=logging.DEBUG,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        datefmt='%m/%d %I:%M:%S',
                        filemode='w')
    logger = logging.getLogger(__name__)

    # basic logger usage
    # logger.debug('This message should go to the log file')
    # logger.info('So should this')
    # logger.warning('And this, too')
    # logger.error('error message')
    # logger.critical('critical message')


    angles = [angle * math.pi/180 for angle in [-45, 0, 45]]
    final_loc = [1000, 1000] # X, Y of the final location

    r = R_Client_Extend(host = "192.168.1.158", 
                        port = 2777,
                        angles = angles)


    r.connect()
    
    dummy_target = Target(target_type = "POS", target_vals = [100, 100])
    r.goto(dummy_target)
 



if __name__ == '__main__':
    main()