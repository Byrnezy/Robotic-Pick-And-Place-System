#!/usr/bin/env python3

import rospy
import math
from nav_msgs.srv import GetPlan
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from scipy import ndimage
import numpy as np
import functools

class FrontierMapping:

    def __init__(self):
        rospy.init_node("frontier_mapping")
        rospy.Subscriber('/map', OccupancyGrid, self.update_map)
        self.FRONTIER_SIZE_TRESHOLD = 2
        self.mapdata = None
        self.frontier_cells = rospy.Publisher('/frontier_mapping/frontier_cells', GridCells, queue_size=10)
        self.centroids = rospy.Publisher('/frontier_mapping/centroids', GridCells, queue_size=10)
        self.frontier = rospy.Publisher('/frontier_mapping/frontiers', Path, queue_size=10)
        rospy.sleep(1.0)
        rospy.loginfo("Frontier mapping node ready")

    def update_map(self, mapdata):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        self.mapdata = mapdata
        self.frontier_detection()

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
        return (y, x)

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
    def neighbors_of_8(self, mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        neighbors = [(x-1, y-1), (x, y-1), (x+1, y-1), (x-1, y), (x+1, y), (x-1, y+1), (x, y+1), (x+1, y+1)]

        return neighbors
    
    def get_frontier_cells(self):
        width = self.mapdata.info.width
        height = self.mapdata.info.height
        map_array = np.array(self.mapdata.data)

        # reshape into 2D map
        map_matrix = np.reshape(map_array, (height, width))

        # use sobel to detect large differences between cells
        sx = ndimage.sobel(map_matrix, axis=0, mode='constant')
        sy = ndimage.sobel(map_matrix, axis=1 , mode='constant')
        sob = np.hypot(sx, sy)

        # create filter to remove edges that border boundaries
        footprint = np.array([[1,1,1],
                              [1,0,1],
                              [1,1,1]])

        def boundary_helper(values):
            return 100 in values
        
        boundary_neighbors = ndimage.generic_filter(map_matrix, boundary_helper, footprint=footprint)
        
        # filter
        indices = np.where((sob >= 1) & (map_matrix == 0) & (boundary_neighbors == 0))
        
        # convert to array of tuple coordinates
        indices = np.array(indices)
        indices = indices.T
        return list(map(tuple, indices))

    def publish_grid_cells(self, cells, publisher):
        # create and publish grid cells message
        world_cells = [self.grid_to_world(self.mapdata, i[1], i[0]) for i in cells]

        resolution = self.mapdata.info.resolution
        msg_grid_cells = GridCells()
        msg_grid_cells.header.stamp = rospy.Time.now()
        msg_grid_cells.header.frame_id = "map"
        msg_grid_cells.cell_height = resolution
        msg_grid_cells.cell_width = resolution
        msg_grid_cells.cells = world_cells

        publisher.publish(msg_grid_cells)

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
            pose_stamped.pose.position = self.grid_to_world(mapdata, point[1], point[0])

            poses.append(pose_stamped)
        
        return poses

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        path_msg.poses = self.path_to_poses(self, mapdata, path)
        self.frontier.publish(path_msg)
        return path_msg

    def segment_frontiers(self, cells):
        frontiers = []
        world_cells = [self.grid_to_world(self.mapdata, i[1], i[0]) for i in cells]

        frontier_count = 0
        while len(cells) > 0:
            # search neighboring cells to build frontier
            frontiers.append([])
            search_queue = []
            search_queue.append(cells.pop())
            frontiers[frontier_count] = []
            while len(search_queue) > 0:
                cell = search_queue.pop()
                frontiers[frontier_count].append(cell)
                x = cell[0]
                y = cell[1]
                neighbors = self.neighbors_of_8(self, self.mapdata, x, y)
                for neighbor in neighbors:
                    if neighbor in cells:
                        search_queue.append(neighbor)
                        cells.remove(neighbor)
            frontier_count += 1
        
        # filter frontiers by size
        filtered_frontiers = [frontier for frontier in frontiers if len(frontier) > self.FRONTIER_SIZE_TRESHOLD]
        return filtered_frontiers

    def get_centroids(self, frontiers):
        centroids = []
        # calculate centroid
        for frontier in frontiers:
            size = len(frontier)
            # average x components
            x = functools.reduce(lambda a, b: a+b, [point[0] for point in frontier]) / size
            # average y components
            y = functools.reduce(lambda a, b: a+b, [point[1] for point in frontier]) / size
            
            # find nearest walkable cell to centroid
            def calc_dist_from_centroid(cell):
                return math.sqrt(math.pow(cell[0] - x, 2) + math.pow(cell[1] - y, 2))
            
            frontier.sort(key=calc_dist_from_centroid)
            centroids.append(frontier[0])
        return centroids

    def frontier_detection(self):
        cell_indices = self.get_frontier_cells()

        self.publish_grid_cells(cell_indices, self.frontier_cells)

        # group nearby frontier cells into arrays
        frontiers = self.segment_frontiers(cell_indices)

        # calculate centroids
        frontier_centroids = self.get_centroids(frontiers)
        print(frontier_centroids)
        self.publish_grid_cells(frontier_centroids, self.centroids)

        # publish list of centroids
        self.path_to_message(self.mapdata, frontier_centroids)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        # if(self.mapdata):
        #     self.frontier_detection()
        rospy.spin()



if __name__ == '__main__':
    FrontierMapping().run()
