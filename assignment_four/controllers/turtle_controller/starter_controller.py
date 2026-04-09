"""student_controller controller."""

import heapq
import math
import numpy as np

MAP_CENTER_INDEX = 5
MAX_A_STAR_SEARCH_STEPS = 10
CELL_SIZE = 0.5

class StudentController:
    def __init__(self):
        self._calculated_path = []
        self._path_waypoint_num = 0
        self.velocity_magnitude = 5
        pass

    def step(self, sensors):
        """
        Compute robot control as a function of sensors.

        Input:
        sensors: dict, contains current sensor values.

        Output:
        control_dict: dict, contains control for "left_motor" and "right_motor"
        """
        control_dict = {"left_motor": 0.0, "right_motor": 0.0}

        # TODO: add your controllers here.
        # print("=========================================================")
        # print("pose: ", sensors["pose"])
        # print("map: ", sensors["map"])
        # print("goal: ", sensors["goal"])
        # print("obstacles: ", sensors["obstacles"])
        # row, col = self.world2cell(sensors["pose"])
        # print("hello")
        # print(f"curr_row: {row}, curr_col: {col}, map[row][col]: {sensors["map"][row][col]}")
        # print(sensors["map"][1][2])

        map = sensors["map"]
        robot_pose = sensors["pose"]
        goal = sensors["goal"]
        obstacles = sensors["obstacles"]
        if len(self._calculated_path) == 0:
            # calculate path needed to get there
            self._calculated_path = self.planner(map,robot_pose, goal, obstacles)
            print(self._calculated_path)
            control_dict["left_motor"] = 0
            control_dict["right_motor"] = 0
        else:
            # print("controls")
            # need to do controls
            next_waypoint = self._calculated_path[self._path_waypoint_num] # find the next waypoint

            if self.euclidean_dist(next_waypoint, robot_pose) < .05: # if close to it, then go to next waypoint
                if self._path_waypoint_num >= len(self._calculated_path):
                    control_dict["left_motor"] = 0
                    control_dict["right_motor"] = 0
                    return control_dict
                self._path_waypoint_num += 1
                next_waypoint = self._calculated_path[self._path_waypoint_num]
            
            dx = next_waypoint[0] - robot_pose[0]
            dy = next_waypoint[1] - robot_pose[1]
            target_angle = math.atan2(dy, dx)
            error = math.atan2(math.sin(target_angle - robot_pose[2]), math.cos(target_angle - robot_pose[2]))

            turn = self.velocity_magnitude * error * 1.5
            forward = self.velocity_magnitude * .5

            left = forward - turn
            right = forward + turn

            control_dict["left_motor"] = left
            control_dict["right_motor"] = right

        return control_dict

# Below are all for calculating the path
    def world2cell(self, pose):
        """
        Finds the cell the robot is in given the pose

        Input:
        pose: pose of where the robot is in

        Output:
        row: row that the robot is in
        col: column that the robot is in 
        """
        
        row = MAP_CENTER_INDEX - int(round(pose[1] / CELL_SIZE))
        col = MAP_CENTER_INDEX + int(round(pose[0] / CELL_SIZE))
        return row, col

    def cell2world(self, pose):
            x = (pose[1] - MAP_CENTER_INDEX) * CELL_SIZE
            y = -(pose[0] - MAP_CENTER_INDEX) * CELL_SIZE
            return x, y

    def planner(self, map, pose, goal, obstacles):
        # contains all the planning steps 
        robot_cell = self.world2cell(pose)
        goal_cell = self.world2cell(goal)
        
        calc_path = self.A_star_search(map.copy(), robot_cell, goal_cell)
        obj_avoidance_path = self.planned_obstacle_avoidance(calc_path, obstacles)



        self.visualize(map, obstacles, pose, goal, obj_avoidance_path)

        return obj_avoidance_path    

    def A_star_search(self, map, robot_cell, goal_cell):
        """
        Notes: g is for steps taken to get to certain cell

        """
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(robot_cell, goal_cell), 0, robot_cell))  # (f, g, node)
        came_from = {}  # to reconstruct path
        g_score = {robot_cell: 0}

        while open_set:
            _, current_g, current_cell = heapq.heappop(open_set)

            if current_cell == goal_cell:
                # reconstruct path
                path = [current_cell]
                while current_cell in came_from:
                    current_cell = came_from[current_cell] # if we can go back more
                    path.append(current_cell)
                path = path[::-1] # swaps the order as path is from goal to start, so need to go in reverse order for start to goal

                # --- Convert path to world coordinates before returning ---
                world_path = [self.cell2world(cell) for cell in path]
                return world_path

            # go through each neighboring 
            for neighbor in self.get_neighbors(map, current_cell):
                neighbor_g = current_g + 1 # neighbor takes one more step than the one it came from
                if neighbor not in g_score or neighbor_g < g_score[neighbor]:
                    # neighbor not in g_score checks to see if we have traversed in this cell before, if not, then we try it out
                    # neighbor_g < g_score[neighbor] checks if you going from current cell is actually faster than the other method in which we have travelled to this cell before
                    g_score[neighbor] = neighbor_g
                    f_new = neighbor_g + self.heuristic(neighbor, goal_cell) # calculate the new traverse path + heuristic
                    heapq.heappush(open_set, (f_new, neighbor_g, neighbor))
                    came_from[neighbor] = current_cell

        return None  # no path found

    def heuristic(self, proposed_cell, goal):
        # going to use Manhattan distance
        return abs(proposed_cell[0] - goal[0]) + abs(proposed_cell[1] - goal[1])
    
    def get_neighbors(self, map, current_cell):
        neighbors = []
        directions = [(-1,0), (1,0), (0,-1), (0,1)]
        max_rows, max_cols= len(map), len(map[0]) # get the max size the neighbor can be in
        for dir_row, dir_col in directions:
            n_row, n_col = current_cell[0] + dir_row, current_cell[1] + dir_col # the cell of the proposed neighbor
            if 0 <= n_row < max_rows and 0 <= n_col < max_cols and map[n_row][n_col] == 0:
                neighbors.append((n_row, n_col))
        return neighbors

    def planned_obstacle_avoidance(self, planned_path, obstacles):

        def draw_circle_points(prev_coor, curr_coor, next_coor, obstacle):
            points = []

            # --- Convert to world ---
            # Assuming coor variables are the (row, col) tuples
            px, py = prev_coor
            cx, cy = curr_coor
            nx, ny = next_coor
            ox, oy = obstacle[0], obstacle[1]

            # --- Radius ---
            radius = math.sqrt(0.2**2 + 0.2**2)

            # --- Angles from obstacle ---
            angle_prev = math.atan2(py - oy, px - ox)
            angle_next = math.atan2(ny - oy, nx - ox)

            # --- Normalize angle difference ---
            diff = angle_next - angle_prev
            diff = math.atan2(math.sin(diff), math.cos(diff))  # wrap to [-pi, pi]

            # --- Decide number of points ---
            if abs(diff) < math.pi / 4:
                num_points = 6   # straight-ish → more points
            else:
                num_points = 4   # sharp turn → fewer points

            # --- Generate arc points ---
            for i in range(1, num_points + 1):
                t = i / (num_points + 1)
                angle = angle_prev + t * diff

                x = ox + radius * math.cos(angle)
                y = oy + radius * math.sin(angle)

                points.append((x, y)) 

            return points



        new_planned_path = []
        new_plan_idx = -1
        for curr_idx in range(len(planned_path)):
            curr_point = planned_path[curr_idx]
            has_obstacle = False
            for obstacle in obstacles:
                if self.euclidean_dist(curr_point, obstacle) <= .28: # know the exact distance it would be as .125, but just made it .2 cause why not
                    # this means that there is an obstacle in this cell
                    
                    # first figure out the previous state from new_planned_path and next state which one planned path knows

                    # first check if new_planned_path has anything
                    if new_plan_idx == -1:
                        break # nothing, so don't need to check this case
                    
                    # make sure there is a next cell
                    if not curr_idx + 1 < len(planned_path):
                        break # do nothing as we just going foward is probably the closest we can get to end goal
                    new_points = draw_circle_points(new_planned_path[new_plan_idx], curr_point, planned_path[curr_idx + 1], obstacle)
                    new_plan_idx += len(new_points)
                    new_planned_path.extend(new_points)
                    has_obstacle = True
                    break # can't have more than one obstacle in there

            # if no obstacle, then just go to the cell
            if not has_obstacle:
                new_planned_path.append(curr_point)
                new_plan_idx+=1 # increment new_planned as there is a new cell added

        return new_planned_path

    
    def euclidean_dist(self, coor1, coor2):
        return math.sqrt((coor1[0]-coor2[0])**2 + (coor1[1]-coor2[1])**2)


    def visualize(self, map_grid, obstacles, pose=None, goal=None, path=None):
        import matplotlib.pyplot as plt

        OBSTACLE_SIZE = 0.25
        fig, ax = plt.subplots()
        rows, cols = len(map_grid), len(map_grid[0])

        # --- Draw map --- 
        for r in range(rows):
            for c in range(cols):
                x_center = (c - MAP_CENTER_INDEX) * CELL_SIZE
                y_center = -(r - MAP_CENTER_INDEX) * CELL_SIZE
                x = x_center - CELL_SIZE / 2
                y = y_center - CELL_SIZE / 2
                rect = plt.Rectangle((x, y), CELL_SIZE, CELL_SIZE, fill=(map_grid[r][c]==1))
                ax.add_patch(rect)

        # --- Draw obstacles ---
        for obs in obstacles:
            obs_x, obs_y = obs
            x = obs_x - OBSTACLE_SIZE / 2
            y = obs_y - OBSTACLE_SIZE / 2
            rect = plt.Rectangle((x, y), OBSTACLE_SIZE, OBSTACLE_SIZE)
            ax.add_patch(rect)

        # --- Draw path (already in world coordinates) ---
        if path is not None:
            xs, ys = zip(*path)
            ax.scatter(xs, ys)
            ax.plot(xs, ys)

        # --- Robot and Goal ---
        if pose is not None:
            ax.plot(pose[0], pose[1], marker='x', markersize=10)
        if goal is not None:
            ax.plot(goal[0], goal[1], marker='*', markersize=10)

        ax.set_aspect('equal')
        ax.set_xlabel("X (world)")
        ax.set_ylabel("Y (world)")
        ax.set_title("Planner Visualization")

        limit = MAP_CENTER_INDEX * CELL_SIZE
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        plt.grid(True)
        plt.pause(0.001)


        


    

    
