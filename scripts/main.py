
import sys
import os

cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))

from udpclient import RClient
import time

import logging

global logger


### Basic usage:


# r=RClient("192.168.1.151",2777)
# r.drive(480,480)
# print(r.sense())

# time.sleep(2)
# r.terminate()

def some_fnc():
    global logger

    logger.info("Nested info")


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
    logger.debug('This message should go to the log file')
    logger.info('So should this')
    logger.warning('And this, too')
    logger.error('error message')
    logger.critical('critical message')



    print("hello")
    some_fnc()




if __name__ == '__main__':
    main()