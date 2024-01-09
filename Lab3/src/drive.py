#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Point, Pose, PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Drive:

    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node('drive')
        
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)
        
        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to_path)

        # Store pose of robot
        self.px = 0
        self.py = 0
        self.pth = 0

        rospy.sleep(1)

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### REQUIRED CREDIT
        ### Make a new Twist message
        msg_cmd_vel = Twist()
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed
        
        ### Publish the message
        self.cmd_vel.publish(msg_cmd_vel)
        
        
    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        start_px, start_py, start_pth = self.px, self.py, self.normalize_angle(self.pth)
        kP = 0.5
        kD = 0
        target_x, target_y = distance*math.cos(start_pth) + start_px, distance*math.sin(start_pth) + start_py
        angle2target = math.atan2(target_y - self.py, target_x - self.px)
        prev_error = self.normalize_angle(self.pth - angle2target)
        while distance - math.sqrt(math.pow(self.px - start_px, 2) + math.pow(self.py - start_py, 2)) > 0.01:
            angle2target = math.atan2(target_y - self.py, target_x - self.px)
            error = self.normalize_angle(angle2target - self.pth)

            deriv_error = error - prev_error
            angular_speed = (error*kP)+(deriv_error*kD) 
            
            self.send_speed(linear_speed, angular_speed)
            
            prev_error = error
            rospy.sleep(0.05)
        
        self.send_speed(0, 0)

    def normalize_angle(self, angle):
        return (angle+math.pi)%(2*math.pi) - math.pi

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        start_angle = self.pth
        goal_angle = self.normalize_angle(angle)
        
        while abs(self.normalize_angle(self.pth - start_angle) - goal_angle) > 0.05:
            print("start_angle: ", start_angle)
            print("self.pth: ", self.normalize_angle(self.pth))
            print("change: ", self.normalize_angle(self.pth - start_angle))
            print("goal_angle: ", goal_angle)
            print("------------------------------------------")
            if(angle <= 0):          
                self.send_speed(0, -aspeed)
            else:
                self.send_speed(0, aspeed)

            rospy.sleep(0.05)
        
        self.send_speed(0, 0)


    def go_to(self, pose):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param pose [PoseStamped] The target pose.
        """
        ### REQUIRED CREDIT
        # pose = pose.pose
        angle = math.atan2(pose.position.y-self.py, pose.position.x-self.px) - self.pth
        if(angle > math.pi):
            angle = angle-(2*math.pi)
        elif(angle < -math.pi):
            angle = angle+(2*math.pi)
        print("angle: ", angle)
        self.rotate(angle, 0.4)
        
        distance = math.sqrt(abs(pose.position.y-self.py)**2+abs(pose.position.x-self.px)**2)
        print("distance: ", distance)
        self.drive(distance, 0.1)
        
        # angle2 = math.pi*pose.orientation.z - self.pth
        # if(angle2 > math.pi):
        #     angle2 = angle2-(2*math.pi)
        # elif(angle2 < -math.pi):
        #     angle2 = angle2+(2*math.pi)
        # print("angle2: ", angle2)
        # self.rotate(angle2, 0.4)

    def get_pose_stamped(self, x, y):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = Pose()

        point = Point()
        point.x = x
        point.y = y 
        point.z = 0
        
        pose_stamped.pose.position = point

        return pose_stamped
    
    def go_to_path(self, msg):
        rospy.wait_for_service('plan_path')
        try:
            get_plan = rospy.ServiceProxy('plan_path', GetPlan)

            current_pose_stamped = self.get_pose_stamped(self.px, self.py)
            
            end_pose_stamped = self.get_pose_stamped(msg.pose.position.x, msg.pose.position.y)
            print("hi")
            path = get_plan(current_pose_stamped, end_pose_stamped, 1)
            print("bye")
            print(path.plan)
            print(path.plan.poses)
            for pose_stamped in path.plan.poses:
                self.go_to(pose_stamped.pose)
        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        ### REQUIRED CREDIT
        # TODO
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        self.pth = yaw

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    Drive().run()
