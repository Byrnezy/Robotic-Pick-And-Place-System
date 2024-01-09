#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Bool
from nav_msgs.srv import GetPlan
from nav_msgs.msg import GridCells, OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Point, Pose, PoseStamped


class Main:

    def __init__(self):
        rospy.init_node("main")
        
        self.drive = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/frontier_mapping/frontiers', Path, self.update_frontier)
        self.frontiers = None
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        rospy.Subscriber('/drive/is_driving', Bool, self.update_is_driving)
        self.is_driving = False
        rospy.Subscriber('/path_planner/invalid', Bool, self.update_invalid_path)
        self.invalid_path = False
        # Store pose of robot
        self.px = 0
        self.py = 0

        rospy.sleep(1.0)
        rospy.loginfo("main node ready")

    def update_frontier(self, frontiers):
        print("New frontiers: ", frontiers)
        self.frontiers = frontiers.poses

    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

    def update_is_driving(self, msg):
        self.is_driving = msg.data
    
    def update_invalid_path(self, msg):
        self.invalid_path = msg.data

    def sorted_frontiers(self):
        def calc_dist(frontier):
            x = frontier.pose.position.x
            y = frontier.pose.position.y
            return math.sqrt(math.pow(x-self.px, 2) + math.pow(y-self.py, 2))
        
        frontiers = self.frontiers
        frontiers.sort(key=calc_dist)
        return frontiers

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.sleep(7)
        #save initial pose
        if(self.frontiers):
            while len(self.frontiers) > 0:
                sorted_frontiers = self.sorted_frontiers()
                while len(sorted_frontiers) > 0:
                    self.drive.publish(sorted_frontiers[0])
                    self.is_driving = True
                    self.invalid_path = False
                    # wait until driving is done
                    while self.is_driving:
                        if(self.invalid_path):
                            break
                        else:
                            rospy.sleep(1)
                    if not self.invalid_path:
                        break
                    else:
                        sorted_frontiers.pop(0)
                if(len(sorted_frontiers) == 0):
                    break
                # break to recalculate frontiers
                rospy.sleep(2)
            
        rospy.spin()



if __name__ == '__main__':
    Main().run()
