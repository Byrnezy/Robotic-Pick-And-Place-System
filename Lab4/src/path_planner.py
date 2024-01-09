#!/usr/bin/env python3

import math
import rospy
from priority_queue import PriorityQueue
from std_msgs.msg import Bool
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        rospy.Service("plan_path", GetPlan, self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.c_space_publisher = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        self.c_space = []
        self.C_SPACE_PADDING = 2
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.astar_expanded = rospy.Publisher('/path_planner/astar_expanded_cells', GridCells, queue_size=10)
        self.astar_frontier = rospy.Publisher('/path_planner/astar_frontier', GridCells, queue_size=10)
        self.path_test = rospy.Publisher('/path_planner/path_test', Path, queue_size=10)

        self.invalid_path = rospy.Publisher('/path_planner/invalid', Bool, queue_size=10)
        
        rospy.Subscriber('/map', OccupancyGrid, self.update_map)
        self.mapdata = None
        
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")



    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        return y * mapdata.info.width + x 
    
    @staticmethod
    def index_to_grid(mapdata, i):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell index.
        :return  ([int], [int]) The (x,y) coordinate .
        """
        x = i % mapdata.info.width
        y = i // mapdata.info.width
        return (x, y)

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        return math.sqrt(math.pow(x2-x1, 2) + math.pow(y2-y1, 2))
        


    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        point = Point()
        resolution = mapdata.info.resolution
        origin = mapdata.info.origin
        point.x = (x + 0.5) * resolution + origin.position.x
        point.y = (y + 0.5) * resolution + origin.position.y
        point.z = 0.0
        return point
        


        
    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        resolution = mapdata.info.resolution
        origin = mapdata.info.origin
        x = int((wp.x - origin.position.x) / resolution)
        y = int((wp.y - origin.position.y) / resolution)
        return (x, y) 

        
    @staticmethod
    def path_to_poses(self, mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        poses = []
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose = Pose()
            pose_stamped.pose.position = self.grid_to_world(mapdata, point[0], point[1])

            poses.append(pose_stamped)
        
        return poses

    

    @staticmethod
    def is_cell_walkable(self, mapdata, x, y):
        width  = mapdata.info.width
        height  = mapdata.info.height
        map_array = mapdata.data
        index = self.grid_to_index(mapdata,x,y)
        boundary_value = 0
        if(index <= len(map_array) - 1):
            boundary_value = map_array[self.grid_to_index(mapdata,x,y)]
        else:
            boundary_value = -1

        if(x < 0 or y < 0 or x > width or y > height or boundary_value == 100 or boundary_value < 0):
            return False
        
        return True

               

    @staticmethod
    def neighbors_of_4(self, mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        walkable_neighbors = [(x, y-1), (x-1, y), (x+1, y), (x, y+1)]

        return filter(lambda cell: self.is_cell_walkable(self, mapdata, cell[0], cell[1]), walkable_neighbors)
    
    @staticmethod
    def neighbors_of_8(self, mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        walkable_neighbors = [(x-1, y-1), (x, y-1), (x+1, y-1), (x-1, y), (x+1, y), (x-1, y+1), (x, y+1), (x+1, y+1)]
        
        return filter(lambda cell: self.is_cell_walkable(self, mapdata, cell[0], cell[1]), walkable_neighbors)
    
    
    def update_map(self, mapdata):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        self.mapdata = mapdata
        print(len(self.mapdata.data))
        self.calc_cspace(mapdata, self.C_SPACE_PADDING)



    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        ## Go through each cell in the occupancy grid
        
        map_array = list(mapdata.data)
        cells_to_dialate_index = []
        cells_to_dialate = []
        for i in range(padding):
            index = 0
            for cell in map_array:
                # find boundary
                if cell == 100:
                    # find cells bordering boundary
                    centerx, centery = self.index_to_grid(mapdata,index)
                    dilated_cells = self.neighbors_of_8(self, mapdata, centerx, centery)
                    
                    # add valid neighbors to the list of cells to dialate
                    for neighbor in dilated_cells:
                        x = neighbor[0]
                        y = neighbor[1]
                        windex = self.grid_to_index(mapdata, x, y)
                        
                        if(windex <= len(map_array)-1):
                            cells_to_dialate_index.append(windex)
                            cells_to_dialate.append(self.grid_to_world(mapdata, x, y))
                                                
                index += 1
            
            # modify temporary map array to add borders before next layer of padding
            for cell in cells_to_dialate_index:
                map_array[cell] = 100

        ## Create a GridCells message and publish it
        msg_grid_cells = GridCells()
        msg_grid_cells.header.stamp = rospy.Time.now()
        msg_grid_cells.header.frame_id = "map"
        msg_grid_cells.cell_height =  mapdata.info.resolution
        msg_grid_cells.cell_width =  mapdata.info.resolution
        msg_grid_cells.cells = cells_to_dialate

        self.c_space_publisher.publish(msg_grid_cells)
        ## Return the C-space
        self.c_space = cells_to_dialate_index

    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        map_array = mapdata.data
        print(start)
        self.calc_cspace(mapdata, self.C_SPACE_PADDING)
        frontier = PriorityQueue()
        frontier.put(start,0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None 
        cost_so_far[start] = 0
        expanded_cells = []

        msg_grid_cells = GridCells()
        msg_grid_cells.header.frame_id = "map"
        msg_grid_cells.cell_height =  mapdata.info.resolution
        msg_grid_cells.cell_width =  mapdata.info.resolution
        while not frontier.empty():

            current = frontier.get()
            x = current[0]
            y = current[1]
            expanded_cells.append(self.grid_to_world(mapdata, x, y))

            rospy.sleep(.002)
            
            # publish expanded cells message
            msg_grid_cells.header.stamp = rospy.Time.now()
            msg_grid_cells.cells = expanded_cells
            self.astar_expanded.publish(msg_grid_cells)

            # publish frontier message
            queue = [self.grid_to_world(mapdata, c[1][0], c[1][1]) for c in frontier.get_queue()]

            msg_grid_cells.header.stamp = rospy.Time.now()
            msg_grid_cells.cells = queue
            self.astar_frontier.publish(msg_grid_cells)

            if current == goal:
                break
            
            adjacent_cells = self.neighbors_of_4(self, mapdata, x, y)
            adjacent_cells = [c for c in adjacent_cells]
            for next in self.neighbors_of_8(self, mapdata, x, y):
                new_cost = cost_so_far[current]
                if(next in adjacent_cells):
                    #print("next: ", next, " is adjacent to ", current)
                    new_cost += 1
                else:
                    new_cost += 1.41

                if self.grid_to_index(mapdata, next[0], next[1]) not in self.c_space and (next not in cost_so_far or new_cost < cost_so_far[next]):
                    cost_so_far[next] = new_cost
                    def calc_heuristic(next):
                        distance = self.euclidean_distance(next[0], next[1], goal[0], goal[1])
                        neighbors = self.neighbors_of_8(self, mapdata, next[0], next[1])
                        cspace_weight = 0
                        print(self.c_space[1])
                        for neighbor in neighbors:
                            if self.grid_to_index(mapdata, neighbor[0], neighbor[1]) in self.c_space:
                                print(next, " is near c_space")
                                cspace_weight += 1
                                break
                        
                        turn_weight = 0
                        if(came_from[current]):
                            previous_angle = math.atan2(current[1] - came_from[current][1], current[0] - came_from[current][0])
                            current_angle = math.atan2(next[1] - current[1], next[0] - current[0])
                            if(previous_angle != current_angle):
                                turn_weight += 1
                        return (distance * 1.5) + (cspace_weight * 5) + turn_weight
                        
                    priority = new_cost + calc_heuristic(next)
                    frontier.put(next, priority)
                    came_from [next]= current
        
        print("DONE SEARCHING")
        path = [goal]
        if(not goal in came_from):
            print("NO GOAL FOUND")
            return None
        else:
            while came_from[path[-1]] is not None:
                path.append(came_from[path[-1]])
            path.reverse()
            return path
            



    
    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        prev_point = path[0]
        to_purge = []
        # Loop through points in the path starting with index 1
        prev_angle = 0
        for i in range(1,len(path)):
            current_point = path[i]
            
            current_angle = math.atan2(current_point[1] - prev_point[1], current_point[0] - prev_point[0])
            
            # compare angle of previous move to current move, if the same, add previous point to purge list
            if(abs(prev_angle - current_angle) < 0.005):
                to_purge.append(path[i-1])

            prev_point = current_point
            prev_angle = current_angle
        # remove all unnecessary nodes
        for i in to_purge:
            path.remove(i)
        
        return(path)

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        path_msg.poses = self.path_to_poses(self, mapdata, path)
        self.path_test.publish(path_msg)
        rospy.loginfo("Returning a Path message")
        return path_msg


        
    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        ## Request the map
        ## In case of error, return an empty path
        if self.mapdata is None:
            return Path()

        ## Execute A*
        start = PathPlanner.world_to_grid(self.mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(self.mapdata, msg.goal.pose.position)
        path  = self.a_star(self.mapdata, start, goal)
        
        if(path):
            ## Optimize waypoints
            path = path[1:]
            waypoints = PathPlanner.optimize_path(path)
            ## Return a Path message
            
            rospy.loginfo(waypoints)
            path_msg = self.path_to_message(self.mapdata, waypoints)
            self.path_test.publish(path_msg)
            return path_msg
        else:
            # No path found send message
            rospy.loginfo("------------------------INVALID PATH-----------------------------")
            bool_msg = Bool()
            bool_msg.data = True
            self.invalid_path.publish(bool_msg)

    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


        
if __name__ == '__main__':
    PathPlanner().run()
