#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Empty as EmptySrv
from nav_msgs.srv import GetPlan
from nav_msgs.msg import GridCells, OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, PoseWithCovarianceStamped


class Main:

    def __init__(self):
        rospy.init_node("main")
        
        self.drive = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.particles_publisher = rospy.Publisher('/particles', PoseArray, queue_size=10)
        self.localize = rospy.Publisher('/localize', Empty, queue_size=10)
        self.done_localizing = rospy.Publisher('/done_localizing', Empty, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        rospy.Subscriber('/path_planner/invalid', Bool, self.update_invalid_path)
        self.invalid_path = False
        rospy.Subscriber('/particlecloud', PoseArray, self.update_particles)
        self.particles = []
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_covariance)
        self.covariance = None
        # Store pose of robot
        self.px = 0
        self.py = 0

        rospy.sleep(1.0)
        rospy.loginfo("main node ready")


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

    def update_invalid_path(self, msg):
        self.invalid_path = msg.data

    def update_particles(self, msg):
        self.particles = msg.poses
        self.particles_publisher.publish(msg)
    
    def update_covariance(self, msg):
        self.covariance = msg.pose.covariance
        rospy.loginfo(self.covariance)
    
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        
        init_particles = rospy.ServiceProxy('global_localization', EmptySrv)
        init_particles()

        self.localize.publish()
        rospy.sleep(5)

        while self.covariance[0] > 0.01:
            pass
        rospy.sleep(5)
        self.done_localizing.publish()
        rospy.spin()



if __name__ == '__main__':
    Main().run()
