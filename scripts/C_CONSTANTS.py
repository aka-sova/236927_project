
# put constants here 


### General
DEBUG = True


### FREQUENCIES

SENSE_UPDATE_FREQ = 0.1
MAP_UPDATE_FREQ = 0.1
MAP_SAVE_FREQ = 0.5
MAP_SAVE_DEBUG_FREQ = 5

# between main and visualization
VIS_LOC_SEND_FREQ = 0.5
VIS_MAP_SEND_FREQ = 1


# vizualization API params
VIS_MAP_OBSTACLES_UPDATE = 0.5


### ALGO PARAMS
MAP_CONNECT_RANGE = 10      # connect the dots in between as containing an obstacle
MAP_INFLATE_RANGE = 10      # 'inflate' the obstacle by this amount to make the routes more safe