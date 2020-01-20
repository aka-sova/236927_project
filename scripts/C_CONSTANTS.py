
# put constants here 


### General
DEBUG = True


### FREQUENCIES

SENSE_UPDATE_FREQ = 0.1         # update the sensors data in the object class
MAP_UPDATE_FREQ = 0.1           # Update the map based on the received measurements
MAP_SAVE_FREQ = 0.5             # Save the map as PICKLE file for the Vizualization GUI
MAP_SAVE_DEBUG_FREQ = 5         # Save the binary, 'connected obstacles', and 'inflated obstacles' maps as images

# between main and visualization
VIS_LOC_SEND_FREQ = 0.5


# vizualization API params
DRAW_REGULAR_OBSTACLES = True
DRAW_INFLATED_OBSTACLES = True

VIS_MAP_OBSTACLES_UPDATE = 0.5      # update the obstacles, shown as pixels
VIS_MAP_BG_UPDATE = 5               # update the background - inflated obstacles




### ALGO PARAMS
MAP_CONNECT_RANGE = 30      # connect the dots in between as containing an obstacle
MAP_INFLATE_RANGE = 10      # 'inflate' the obstacle by this amount to make the routes more safe
