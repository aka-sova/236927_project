

import timeit
import os
import sys
cur_loc = os.getcwd()
sys.path.append(os.path.join(cur_loc, 'scripts\\api_src'))
sys.path.append(os.path.join(cur_loc, 'scripts'))


from api_src.udpclient import RClient
from map_func import Map


import C_CONSTANTS

import numpy as np
import math
import random
import copy
import pickle

from datetime import datetime

import matplotlib.pyplot as plt

class Target(object):
    def __init__(self, target_type : str, target_vals : list):
        self.type = target_type

        self.x = ''
        self.y = ''
        self.angle = ''

        if self.type == "POS":
            self.x = target_vals[0]
            self.y = target_vals[1]
        elif self.type == "ROT":
            self.angle = target_vals[0]
        elif self.type == "POPULATE_MAP":
            pass
        else:
            raise Exception("Invalid target type")


class RRTStar(object):

    class Node:
        """
        Class defining an RRT Node
        """

        def __init__(self, row, col):
            self.row = row
            self.col = col
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0            

    def __init__(self, logger, max_iter, expand_dis, path_resolution, connect_circle_dist, goal_sample_rate, artifacts_loc):

        self.logger = logger

        self.artifacts_loc = artifacts_loc

        self.debug_path_idx = 0


        self.max_iter = max_iter
        self.path_resolution = path_resolution
        self.connect_circle_dist = connect_circle_dist
        self.goal_sample_rate = goal_sample_rate

        self.start = None
        self.end = None
        # self.min_rand = rand_area[0]
        # self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.node_list = []


        self.map = None
        self.map_size_x = None
        self.map_size_y = None
        self.path_nodes_list = []
        self.output_togo_list = []
        self.optimized_path = []

        self.collision_margin = 3


    def find_path(self, init_loc : list, target : Target, map : Map):
        """Use the R* algorithm to find the path to the destination
        Input: 

        init_loc - initial location [X, Y]
        target -    Target  - has x,y
        map      - Map object, containing all the map information

        Output:
        list of GOTO Target objects defining the required path
        """
        self.map = map
        self.map_size_x = map.bin_map.shape[0]
        self.map_size_y = map.bin_map.shape[1]
        self.output_togo_list = []

        init_col, init_row = Map.to_csv_coords(rows = self.map_size_x, 
                                    cols = self.map_size_y, 
                                    x=init_loc[0], 
                                    y=init_loc[1])

        dest_col, dest_row = Map.to_csv_coords(rows = self.map_size_x, 
                                    cols = self.map_size_y, 
                                    x=target.x, 
                                    y=target.y)                        


        self.logger.info("Finding a path from [{} {}] to [{} {}]".format(init_loc[0], init_loc[1], target.x, target.y))



        # 1. Check if the path is obstacle free
        self.start = RRTStar.Node(init_row, init_col)
        self.end = RRTStar.Node(dest_row, dest_col)

        
        if self.check_line_collision(self.start, self.end):
            self.logger.info("No collision spotted, path is free")
            self.path_nodes_list = [self.end]                       # output only the end node
            self.output_togo_list = self.transform_nodes_into_targets(self.path_nodes_list)
            return self.output_togo_list

        else:
            self.logger.info("Collision detected, initializing algorithm")
            # 2.1 if not, generate a tree
            self.node_list = [self.start]
            for i in range(self.max_iter):

                # self.logger.info("Iter: {} ,  number of nodes: {}".format( i, len(self.node_list)))

                rnd = self.get_random_node()
                nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
                new_node = self.steer(from_node = self.node_list[nearest_ind], to_node =  rnd, extend_length = self.expand_dis)

                if self.check_static_collision(new_node):
                    near_inds = self.find_near_nodes(new_node)
                    new_node = self.choose_parent(new_node, near_inds)
                    if new_node:
                        self.node_list.append(new_node)
                        self.rewire(new_node, near_inds)

            
            self.logger.info("Tree construction completed")

            # 2.3 Choose the closest node to the target
            self.logger.info("Finding a route") 


            last_index = self.search_best_goal_node()
            if last_index:
                self.path_nodes_list =  self.generate_final_course(last_index)

            if self.path_nodes_list is not []:
                # remove nodes with no collision
                # 2.2 Optimize the path, connect nodes which have no obstacles in between
                self.optimized_path = self.optimize_path(self.path_nodes_list)
            else:
                self.logger.WARNING("Route was not found! ")                         

        self.draw_graph()
        self.output_togo_list = self.transform_nodes_into_targets(self.optimized_path)

        self.logger.info("Path generation successful")     
        return self.output_togo_list[1:] # don't return the initial node

    
    def check_static_collision(self, node):
        """Check is the node is inside the 'occupied' pixel
        return TRUE if no collision takes place"""

        if self.map.inflated_map[(int)(node.row)][(int)(node.col)] == 0:
            return True
        return False

    def check_line_collision(self, node_start, node_finish):

        if C_CONSTANTS.LINE_OBSTACLE_AVOIDANCE_ALGORITHM == "CROSS":
            return self.check_line_collision_cross(node_start, node_finish)

        if C_CONSTANTS.LINE_OBSTACLE_AVOIDANCE_ALGORITHM == "LINE_BUILDING":
            return self.check_line_collision_line_builing(node_start, node_finish)
      
        if C_CONSTANTS.LINE_OBSTACLE_AVOIDANCE_ALGORITHM == "FILLED":
            return self.check_line_collision_filled(node_start, node_finish)
      

    def check_line_collision_cross(self, node_start, node_finish):
        """Check if the line between the nodes crosses any of the occupied pixels
        return TRUE if path is collision FREE"""

        # collision check is performed using the 'cross' operator

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            start = timeit.default_timer()

        # for each obstacle pixel, check if it intercepts the line between 2 nodes
        obstacles = np.transpose(np.nonzero(self.map.inflated_map))

        # build a line function describing the path from start to finish
        # X = col
        # Y = row

        min_row = min(node_finish.row, node_start.row)
        max_row = max(node_finish.row, node_start.row)
        min_col = min(node_finish.col, node_start.col)
        max_col = max(node_finish.col, node_start.col)

        for obstacle in obstacles:
            nonzero_row = obstacle[0]
            nonzero_col = obstacle[1]

            # 1. check if the obstacle is outside of the region
            if nonzero_row < min_row or nonzero_row > max_row or nonzero_col < min_col or nonzero_col > max_col:
                continue
        
            # 2. check what is the discrepancy of it if we put it into the line function
            p1 = np.array([node_start.col, node_start.row])
            p2 = np.array([node_finish.col, node_finish.row])
            p3 = np.array([nonzero_col,nonzero_row])

            distance = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)

            if distance < self.collision_margin:
                if C_CONSTANTS.PRINT_TIMES_RRT == True:
                    stop = timeit.default_timer()
                    self.logger.info('[RRT] Collision line check. Time elapsed :  {}'.format(stop - start))  

                return False

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            stop = timeit.default_timer()
            self.logger.info('[RRT] Collision line check. Time elapsed :  {}'.format(stop - start))  

        return True

    def check_line_collision_line_builing(self, node_start, node_finish):
        """Check if the line between the nodes crosses any of the occupied pixels
        return TRUE if path is collision FREE"""

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            start = timeit.default_timer()

        # for each obstacle pixel, check if it intercepts the line between 2 nodes
        obstacles = np.transpose(np.nonzero(self.map.inflated_map))

        # build a line function describing the path from start to finish
        # X = col
        # Y = row

        min_row = min(node_finish.row, node_start.row)
        max_row = max(node_finish.row, node_start.row)
        min_col = min(node_finish.col, node_start.col)
        max_col = max(node_finish.col, node_start.col)

        # build the line function

        # find a slope
        d_row = node_finish.row - node_start.row
        d_col = node_finish.col - node_start.col

        if d_col != 0:
            slope_1 = (float)(d_row / d_col)
        else:
            slope_1 = np.inf


        if d_row != 0:
            slope_2 = (float)(d_col / d_row)
        else:
            slope_2 = np.inf
        # build a line

        row_0 = node_finish.row - (slope_1 * node_finish.col)
        col_0 = node_finish.col - (slope_2 * node_finish.row)


        # the equation 1 is 
        #       ROW = SLOPE_1 * COL + ROW_0


        # the equation 2 is 
        #       COL = SLOPE_2 * ROW + COL_0


        # for each obstacle, check how far it is from the line in both directions - X and Y

        for obstacle in obstacles:
            nonzero_row = obstacle[0]
            nonzero_col = obstacle[1]

            # 1. check if the obstacle is outside of the region
            if nonzero_row < min_row or nonzero_row > max_row or nonzero_col < min_col or nonzero_col > max_col:
                continue
        
            # 2. check what is the discrepancy of it if we put it into the line function
            if slope_1 != np.inf:
                calculated_row = slope_1 * nonzero_col + row_0
                dist_1 = np.abs(calculated_row - nonzero_row)

            if slope_2 != np.inf:
                calculated_col = slope_2 * nonzero_row + col_0
                dist_2 = np.abs(calculated_col - nonzero_col)  

            distance = min(dist_1, dist_2)

            if distance < self.collision_margin:
                if C_CONSTANTS.PRINT_TIMES_RRT == True:
                    stop = timeit.default_timer()
                    self.logger.info('[RRT] Collision line check. Time elapsed :  {}'.format(stop - start))  

                return False

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            stop = timeit.default_timer()
            self.logger.info('[RRT] Collision line check. Time elapsed :  {}'.format(stop - start))  

        return True


    def check_line_collision_filled(self, node_start, node_finish):
        """Check if the line between the nodes crosses any of the occupied pixels
        return TRUE if path is collision FREE"""

        # USE THE FILLED MAP AND ADD THE INFLATION AS THE CIRCLES
        # WE'LL HAVE TO CHECK LESS EQUATIONS

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            start = timeit.default_timer()

        # for each obstacle pixel, check if it intercepts the line between 2 nodes
        obstacles = np.transpose(np.nonzero(self.map.filled_map)) # FILLED MAP

        # build a line function describing the path from start to finish
        # X = col
        # Y = row

        min_row = min(node_finish.row, node_start.row)
        max_row = max(node_finish.row, node_start.row)
        min_col = min(node_finish.col, node_start.col)
        max_col = max(node_finish.col, node_start.col)

        for obstacle in obstacles:
            nonzero_row = obstacle[0]
            nonzero_col = obstacle[1]

            # 1. check if the obstacle is outside of the region
            if nonzero_row < min_row or nonzero_row > max_row or nonzero_col < min_col or nonzero_col > max_col:
                continue
        
            # 2. check what is the discrepancy of it if we put it into the line function
            p1 = np.array([node_start.col, node_start.row])
            p2 = np.array([node_finish.col, node_finish.row])
            p3 = np.array([nonzero_col,nonzero_row])

            distance = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)

            if distance < C_CONSTANTS.MAP_INFLATE_RANGE:
                if C_CONSTANTS.PRINT_TIMES_RRT == True:
                    stop = timeit.default_timer()
                    self.logger.info('[RRT] Collision line check. Time elapsed :  {}'.format(stop - start))  

                return False

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            stop = timeit.default_timer()
            self.logger.info('[RRT] Collision line check. Time elapsed :  {}'.format(stop - start))  

        return True

    def get_random_node(self):
        """Generate a new random node"""

        rnd = self.Node(random.uniform(0, self.map_size_x),
                        random.uniform(0, self.map_size_y))




        return rnd


    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        """Calculate minimum distance node. Return the INDEX of the Node with minimal distance"""

        dlist = [(node.row - rnd_node.row) ** 2 + (node.col - rnd_node.col)  ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind


    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.col - from_node.col
        dy = to_node.row - from_node.row
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


    @staticmethod
    def calc_distance(from_node, to_node):
        dx = to_node.col - from_node.col
        dy = to_node.row - from_node.row
        d = math.hypot(dx, dy)
        return d

    def steer(self, from_node, to_node, extend_length=float("inf")):

        """If the node is further than the 'extend_length', bring it closer, but in same direction"""

        new_node = self.Node(to_node.row, to_node.col)
        new_node.parent = from_node
        d, theta = self.calc_distance_and_angle(from_node, to_node)

        if d < extend_length:
            # no need to steer
            new_node.parent = from_node
            return new_node

        else: 
            # steer to a length of the increment
            new_node.col = from_node.col + extend_length * math.cos(theta)
            new_node.row = from_node.row + extend_length * math.sin(theta)

            return new_node

    def find_near_nodes(self, new_node):
        """Find nearby nodes in a certain radius - return INDIXES of those nodes"""

        # nnode = len(self.node_list) + 1
        # r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))  # check this value
        r = self.connect_circle_dist * 1.5  # check this value
        dist_list = [(node.col - new_node.col) ** 2 + (node.row - new_node.row) ** 2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]


        return near_inds


    def choose_parent(self, new_node, near_inds):
        """From all the nodes nearby, choose the node, upon connecting to which, the cost will be MININAL"""
        if not near_inds:
            return None

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            start = timeit.default_timer()

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]

            if self.check_line_collision(near_node, new_node):

                # t_node = self.steer(near_node, new_node)
                # if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node

        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        # new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            stop = timeit.default_timer()
            self.logger.info('[RRT] choose_parent. Time elapsed :  {}'.format(stop - start))  
 
        return new_node        

    def calc_new_cost(self, from_node, to_node):
        """Calculate the cost of the to_node, if it connects to 'from_node'"""
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d


    def rewire(self, new_node, near_inds):
        """Recalculate whether connecting nearby nodes to the 'new_node' will improve their cost"""

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            start = timeit.default_timer()


        for i in near_inds:
            near_node = self.node_list[i]

            no_collision = self.check_line_collision(near_node, new_node)

            if no_collision:
                new_cost = self.calc_new_cost(from_node = new_node, to_node = near_node)
                improved_cost = near_node.cost > new_cost

                if improved_cost:
                    near_node.parent = new_node
                    self.propagate_cost_to_leaves(new_node)        

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            stop = timeit.default_timer()
            self.logger.info('[RRT] rewire. Time elapsed :  {}'.format(stop - start))  
 

    def propagate_cost_to_leaves(self, parent_node):
        """after updating the cost of a node, update cost of all the nodes depending on this node"""
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)               

    def generate_final_course(self, goal_ind):

        self.end.parent = self.node_list[goal_ind]
        path = [self.end]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(node)

        # REVERSE THE LIST
        path.reverse()
        return path

    def search_best_goal_node(self):


        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            start = timeit.default_timer()

         # dist_to_goal_list = [self.calc_distance(n, self.end) for n in self.node_list]

        # goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_dis]

        # from all the nodes, take the one which will:
        #   1.  give the smallest cost 
        #   2.  collision free

        # TODO - check only nodes in a certain radius!!! otherwise takes a lot of time
        
        dist_to_goal_list = [self.calc_distance(n, self.end) for n in self.node_list]

        found_goal_ind = False
        multiplier = 1

        while not found_goal_ind and multiplier < 5:
            goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_dis*multiplier]
            if goal_inds == []:
                multiplier += 1
            else:
                found_goal_ind = True

        safe_node_idsx = []

        # for node_idx, node in enumerate(self.node_list):
        #     if self.check_line_collision(node_start = node, node_finish = self.end):
        #         safe_node_idsx.append(node_idx)

        for node_idx in goal_inds:
            if self.check_line_collision(node_start = self.node_list[node_idx], node_finish = self.end):
                safe_node_idsx.append(node_idx)

        if safe_node_idsx == []:
            return None

        # calculate the costs, get the index of the node with MINIMUM added cost
        min_cost = np.inf
        min_index = None
        for i in safe_node_idsx:
            new_cost =  self.calc_new_cost(from_node = self.node_list[i], to_node = self.end) 

            if new_cost < min_cost:
                min_cost = new_cost
                min_index = i

        if min_index == None:
            raise Exception("Should not happen!")

        if C_CONSTANTS.PRINT_TIMES_RRT == True:
            stop = timeit.default_timer()
            self.logger.info('[RRT] search_best_goal_node. Time elapsed :  {}'.format(stop - start))  
 


        return min_index


    def optimize_path(self, path_list):
        """Optimize path upon the visibility constraint. For every node in the path,
        if the path is obstacle free, connect the nodes directly
        input - list of NODES"""

        # start and end node with remain the same

        path_list_copy = copy.deepcopy(path_list)

        change_done = True

        while change_done:
            change_done, path_list_copy_new = self.optimize_path_once(path_list_copy)
            path_list_copy = copy.deepcopy(path_list_copy_new)
 
        return path_list_copy

    def optimize_path_once(self, path_list):
        """Loop one time over the path"""

        path_list_copy = copy.deepcopy(path_list)
        active_index = 0
        change_done = False

        while (active_index < len(path_list_copy) - 2) :

            if active_index == len(path_list_copy) - 2:
                return change_done, path_list_copy
        
            if self.check_line_collision(node_start = path_list_copy[active_index], node_finish = path_list_copy[active_index+2]):
                # no collision! remove the node in between
                path_list_copy.pop(active_index + 1)
                path_list_copy[active_index+1].parent = path_list_copy[active_index]
                change_done = True
            else:
                active_index += 1

        return change_done, path_list_copy


    def transform_nodes_into_targets(self, nodes_list):


        togo_targets_list = []


        for node in nodes_list:
            x, y = Map.to_x_y_coords(rows = self.map_size_x, cols = self.map_size_y, row = node.row, col = node.col)
            trg_obj = Target(target_type = "POS", target_vals = [x, y])

            togo_targets_list.append(trg_obj)

        return togo_targets_list

    def draw_graph(self):

        """Create a plot of the path found and save it"""

        plt.clf()

        for node in self.node_list:
            if node.parent:
                plt.plot([node.parent.col, node.col], [node.parent.row, node.row], "-g")

        # rad_list = [50, 80]
        # centers_list = [[100,100], [250,250]]


        # for radius, center_coords in zip(rad_list, centers_list):
        #     self.plot_circle(center_coords[0], center_coords[1], radius)

        # plot the origin & target 
        plt.plot(self.start.col, self.start.row, "xr")
        plt.plot(self.end.col, self.end.row, "xr")
        plt.axis("equal")
        plt.axis([0, self.map_size_x, self.map_size_y, 0]) # X axis is inverted
        plt.grid(True)

        # plot the obstacles
        obstacles = np.transpose(np.nonzero(self.map.inflated_map))

        for obstacle in obstacles:
            nonzero_row = obstacle[0]
            nonzero_col = obstacle[1]
            plt.plot(nonzero_col, nonzero_row, ",k")

        # plot the original path
        for node in self.path_nodes_list:
            if node.parent:
                plt.plot([node.parent.col, node.col], [node.parent.row, node.row], 'ro--', linewidth=2)

        # plot the optimized path
        for node in self.optimized_path:
            if node.parent:
                plt.plot([node.parent.col, node.col], [node.parent.row, node.row], 'ko--', linewidth=2)

        debug_name = os.path.join(self.artifacts_loc, (str)(self.debug_path_idx) + '.png')
        plt.savefig(debug_name)
        self.debug_path_idx += 1
        

    @staticmethod
    def plot_circle(row, col, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [col + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [row + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)