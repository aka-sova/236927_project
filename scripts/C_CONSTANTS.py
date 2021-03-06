
# put constants here 


### General
DEBUG = True
PRINT_TIMES_MAP = False
PRINT_TIMES_RRT = False


### FREQUENCIES

SENSE_UPDATE_FREQ = 0.1         # update the sensors data in the object class
MAP_UPDATE_FREQ = 0.1           # Update the map based on the received measurements
MAP_SAVE_FREQ = 0.5             # Save the map as PICKLE file for the Vizualization GUI
MAP_SAVE_DEBUG_FREQ = 5         # Save the binary, 'connected obstacles', and 'inflated obstacles' maps as images

# between main and visualization
VIS_LOC_SEND_FREQ = 0.5         # Send data between Main and Visualization


# vizualization API params
DRAW_REGULAR_OBSTACLES  = True
DRAW_INFLATED_OBSTACLES = False
DRAW_AGENT_PATH         = True

VIS_MAP_OBSTACLES_UPDATE = 0.5      # update the obstacles, shown as pixels
VIS_MAP_BG_UPDATE = 5               # update the background - inflated obstacles
VIS_MAP_PATH_UPDATE = 0.2           # it will check for the 'updated_path.txt' file with this frequency

### DYNAMICS_PARAMS
REACH_MARGIN = 10                   # Check reached the target
MAP_UPDATE_WAIT_TIME = 1            # Checking the Clear Path before movement. Wait for this amount to ensure map is updated

PERFORM_ANGLE_VALIDATION_EN_ROUTE = False
ACCEPTABLE_TURNING_ANGLE = 5        # in [deg]. If robot looks at target at this angle or smaller, it will start moving
OBSTACLE_COLLISION_MARGIN = 0.5     # in [cm]. Used to find when the robot meets the border (inflated map)
CLEAR_PATH_FINDING_MARGIN = 1.1     # multiplies the distance to drive before validating next movement won't meet any obstacle

### ALGO PARAMS
MAP_CONNECT_RANGE = 30      # connect the dots in between as containing an obstacle
MAP_INFLATE_RANGE = 10      # 'inflate' the obstacle by this amount to make the routes more safe
LINE_OBSTACLE_AVOIDANCE_ALGORITHM = "LINE_BUILDING" # choose from [ "FILLED" / "LINE_BUILDING" / "CROSS" ]

### RRT params
ITERATIONS_NUMBER = 100