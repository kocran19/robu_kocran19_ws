import rclpy

#used to create nodes
from rclpy.node import Node

# Enables the use of the string message type
from std_msgs.msg import String

# Twist is linear and angular velocity
from geometry_msgs.msg import Twist

# Handles LaserScan messages to sense distance to obstacles (i.e. walls)        
from sensor_msgs.msg import LaserScan

# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data 

# Scientific computing library
import numpy as np

from enum import IntEnum

import math

ROBOT_DIRECTION_FRONT_INDEX = 0
ROBOT_DIRECTION_LEFT_FRONT_INDEX = 60
ROBOT_DIRECTION_LEFT_INDEX = 90
ROBOT_DIRECTION_LEFT_REAR_INDEX = 120
ROBOT_DIRECTION_REAR_INDEX = 180
ROBOT_DIRECTION_RIGHT_REAR_INDEX = 240
ROBOT_DIRECTION_RIGHT_INDEX = 270
ROBOT_DIRECTION_RIGHT_FRONT_INDEX = 300

class WallFollowerStates(IntEnum):
    WF_STATE_INVALID = -1,
    WF_STATE_DETECTWALL = 0,
    WF_STATE_DRIVE2WALL = 1,
    WF_STATE_ROTATE2WALL = 2,
    WF_STATE_ALIGNWALL = 3,
    WF_STATE_FOLLOWWALL = 4

class WallFollower(Node):

    def __init__(self):
        super().__init__('WallFollower')
        
        # Create a subscriber
        # This node subscribes to messages of type 
        # sensor_msgs/LaserScan     
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        # Create a publisher
        # This node publishes the desired linear and angular velocity 
        # of the robot (in the robot chassis coordinate frame) to the 
        # /cmd_vel topic.
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
        
        # Initialize the LaserScan sensor readings to some large value
        # Values are in meters.

        self.left_dist = 999999.9 # Left
        self.leftfront_dist = 999999.9 # Left-front
        self.front_dist = 999999.9 # Front
        self.rightfront_dist = 999999.9 # Right-front
        self.right_dist = 999999.9 # Right
        self.rightrear_dist = 999999.9
        self.rear_dist = 999999.9 # Rear
        self.leftrear_dist = 999999.9

        self.distances_history = []
        self.distances_histroy_size = 1

        ############# WALL FOLLOWING MODE PARAMETERS ##################     
        self.wallfollower_state = WallFollowerStates.WF_STATE_INVALID

        # Maximum forward speed of the robot in meters per second
        # Any faster than this and the robot risks falling over.
        self.forward_speed_wf_fast = 0.1
        self.forward_speed_wf_slow = 0.05

        # Set turning speeds (to the left) in rad/s 
        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 1.0  # Fast turn
        self.turning_speed_wf_slow = 0.1 # Slow turn
         
        # Wall following distance threshold.
        # We want to try to keep within this distance from the wall.
        self.dist_thresh_wf = 0.3 # in meters  
        self.dist_hysteresis_wf = 0.02 # in meters

        self.dist_laser_offset = 0.03
        self.minimum_distance_laser = 0.1

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if len(self.distances_history) > 0:
            self.follow_wall()

    def get_dist_avg_history(self, pos):
        sum = 0.0
        number_of_valid_values = 0

        for distances in self.distances_history:
            if (distances[pos] > self.minimum_distance_laser):
                sum = sum + distances[pos]
                number_of_valid_values = number_of_valid_values +1
        
        if (number_of_valid_values > 0):
            return sum / number_of_valid_values
        else:
            return -1
        

    def scan_callback(self, msg):
        """
        This method gets called every time a LaserScan message is 
        received on the /scan ROS topic   
        """
        
        self.distances_history.append(msg.ranges)
        if (len(self.distances_history) > self.distances_histroy_size):
            self.distances_history = self.distances_history[1:]
        
        self.left_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_INDEX)
        self.leftfront_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_FRONT_INDEX)
        self.front_dist = self.get_dist_avg_history(ROBOT_DIRECTION_FRONT_INDEX)
        self.rightfront_dist = self.get_dist_avg_history(ROBOT_DIRECTION_RIGHT_FRONT_INDEX)
        self.right_dist = self.get_dist_avg_history(ROBOT_DIRECTION_RIGHT_INDEX)
        self.rightrear_dist = self.get_dist_avg_history(ROBOT_DIRECTION_RIGHT_REAR_INDEX)
        self.rear_dist = self.get_dist_avg_history(ROBOT_DIRECTION_REAR_INDEX)
        self.leftrear_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_REAR_INDEX)

        # if self.left_dist == np.inf:
        #     self.left_dist = 10
        # if self.leftfront_dist == np.inf:
        #     self.leftfront_dist = 10
        # if self.front_dist == np.inf:
        #     self.front_dist = 10
        # if self.rightfront_dist == np.inf:
        #     self.rightfront_dist = 10
        # if self.right_dist == np.inf:
        #     self.right_dist = 10
        # if self.rear_dist == np.inf:
        #     self.rear_dist = 10

        print("l: %.2f m" % self.left_dist, 
              "lf: %.2f m" % self.leftfront_dist, 
              "f: %.2f m" % self.front_dist, 
              "rf: %.2f m" % self.rightfront_dist, 
              "r: %.2f m" % self.right_dist, 
              "rb: %.2f m" % self.rightrear_dist,
              "b: %.2f m" % self.rear_dist,
              "lb: %.2f m" % self.leftrear_dist
              )
    

    def follow_wall(self):
        """
        This method causes the robot to follow the boundary of a wall and corners.
        """
        # Create a geometry_msgs/Twist message
        
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0        

        if (self.wallfollower_state == WallFollowerStates.WF_STATE_INVALID):
            print("WF_STATE_DETECTWALL")
            self.wallfollower_state = WallFollowerStates.WF_STATE_DETECTWALL

        elif self.wallfollower_state == WallFollowerStates.WF_STATE_DETECTWALL:
            dist_min = min(self.distances_history[-1])
            #turn left to the minimal distance
            if ((self.front_dist) > (dist_min + self.dist_laser_offset)):
                if abs((self.front_dist - dist_min)) < 0.2:
                    msg.angular.z = self.turning_speed_wf_slow
                else:
                    msg.angular.z = self.turning_speed_wf_fast
            else:
                print("WF_STATE_DRIVE2WALL")
                self.wallfollower_state = WallFollowerStates.WF_STATE_DRIVE2WALL
        
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_DRIVE2WALL:
            fd_thresh = self.dist_thresh_wf + self.dist_laser_offset

            forward_speed_wf = self.calc_linear_speed()
            
            if self.front_dist > (fd_thresh + self.dist_hysteresis_wf):
                msg.linear.x = forward_speed_wf
            elif self.front_dist < (fd_thresh - self.dist_hysteresis_wf):
                msg.linear.x = -forward_speed_wf
            else:
                turn_direction = self.align_front()
                msg.angular.z = self.turning_speed_wf_slow  * turn_direction
                if turn_direction == 0:
                    print("WF_STATE_ROTATE2WALL")
                    #save the current distances as input for the state ROTATE2WALL
                    self.wallfollower_state_input_dist = self.distances_history[-1]
                    self.wallfollower_state = WallFollowerStates.WF_STATE_ROTATE2WALL

        elif self.wallfollower_state == WallFollowerStates.WF_STATE_ROTATE2WALL:
            sr = self.wallfollower_state_input_dist[ROBOT_DIRECTION_RIGHT_INDEX] #Abstand Statisch Rechts (vom vorigen State übernommen)

            #print("Min-Abstand: %.2f, lr = %.2f, l = %.2f, lf = %.2f, f = %.2f, rf=%.2f" % (dist_min, lr, self.left_dist, lf, self.front_dist, rf))
        
            # (sr != np.inf) -> Rechts wurde eine Wand erkannt 
            #                -> nur bis zur diese Wand drehen
            # (sr == np.inf) -> Rechts befindet sich keine Wand im Erkennungsbereich des Laser 
            #                -> drehen bis der Frontabstand ebenfalls unendlich anzeigt
            if ((sr != np.inf) and (abs(self.front_dist - self.dist_laser_offset - sr) > 0.05)) or ((self.front_dist != np.inf) and (sr == np.inf)):
                msg.angular.z = -self.turning_speed_wf_fast
            else:
                turn_direction = self.align_left()
                msg.angular.z = self.turning_speed_wf_slow * turn_direction
                if turn_direction == 0:
                    print("WF_STATE_FOLLOWWALL")
                    self.wallfollower_state = WallFollowerStates.WF_STATE_FOLLOWWALL   

            # # right corner
            # if (abs(sl - self.dist_thresh_wf) < self.dist_hysteresis_wf) and (abs(sf - self.dist_laser_offset - self.dist_thresh_wf) < self.dist_hysteresis_wf) and (sr > sf):
            #     if (self.front_dist - self.dist_laser_offset - sr) < 0:
            #         msg.angular.z = -self.turning_speed_wf_slow
            # # robot faces to a wall
            # elif (abs(sf - self.dist_laser_offset - self.dist_thresh_wf) < self.dist_hysteresis_wf) and (sr > self.dist_thresh_wf):
            #     if (self.front_dist - self.dist_laser_offset - sr) < 0:
            #         msg.angular.z = -self.turning_speed_wf_slow    
            # else:
            #     pass    # left corner -> 
            
        elif self.wallfollower_state == WallFollowerStates.WF_STATE_FOLLOWWALL:
            fd_thresh = self.dist_thresh_wf + self.dist_laser_offset
            rd_thresh = self.dist_thresh_wf

            lf = self.leftfront_dist
            lr = self.leftrear_dist

            forward_speed_wf = self.calc_linear_speed()

            if (self.front_dist > (fd_thresh + self.dist_hysteresis_wf)):
                #Nach Links lenken -> Abstand wurde zu groß
                if self.left_dist > (rd_thresh + self.dist_hysteresis_wf):
                    #Verhindert ein Aufschwingen, 
                    #Vergleichswert self.dist_hysteresis_wf ist nicht optimal
                    #-> eigenen Parameter erstellen und Winkel vergleichen!
                    if (lr - lf) < self.dist_hysteresis_wf:
                        msg.angular.z = self.turning_speed_wf_slow
                    msg.linear.x = forward_speed_wf
                #Nach Rechts lenken -> Abstand wurde unterschritten
                elif self.left_dist < (rd_thresh - self.dist_hysteresis_wf):
                    if (lf - lr) < self.dist_hysteresis_wf:
                        msg.angular.z = -self.turning_speed_wf_slow
                    msg.linear.x = forward_speed_wf
                #Geradeaus fahren
                else:
                    msg.linear.x = forward_speed_wf
            else: #Wand oder Ecke erreicht!
                turn_direction = self.align_left()
                msg.angular.z = self.turning_speed_wf_slow * turn_direction
                if turn_direction == 0:
                    print("WF_STATE_ROTATE2WALL")
                    self.wallfollower_state_input_dist = self.distances_history[-1]
                    self.wallfollower_state = WallFollowerStates.WF_STATE_ROTATE2WALL
        else:
            self.wallfollower_state = WallFollowerStates.WF_STATE_DETECTWALL

        # Send velocity command to the robot
        self.cmd_vel_publisher.publish(msg)
    

    def align_front(self):
        fl = self.distances_history[-1][ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        fr = self.distances_history[-1][ROBOT_DIRECTION_RIGHT_FRONT_INDEX]
            
        if ( (fr - fl) > self.dist_hysteresis_wf ):
            return 1    #turning left
        elif ( (fl - fr) > self.dist_hysteresis_wf ):
            return -1   #truning right
        else:
            return 0    #aligned

    def align_left(self):
        lf = self.distances_history[-1][ROBOT_DIRECTION_LEFT_FRONT_INDEX]
        lr = self.distances_history[-1][ROBOT_DIRECTION_LEFT_REAR_INDEX]

        if (lf - lr) > self.dist_hysteresis_wf:
            return 1    #turning left
        elif (lr - lf) > self.dist_hysteresis_wf:
            return -1   #truning right
        else:
            return 0    #aligned

    def calc_angular_speed(self):
        pass

    def calc_linear_speed(self):
        fd_thresh = self.dist_thresh_wf + self.dist_laser_offset
        if self.front_dist > (1.2 * fd_thresh):
            forward_speed_wf = self.forward_speed_wf_fast
        else:
            forward_speed_wf = self.forward_speed_wf_slow

        return forward_speed_wf
    

    def calc_left_wall_angle(self):

        LEFT_ANGLE_DEG = 10
        LEFT_ANGLE_RAD =   (math.pi/180.0) * LEFT_ANGLE_DEG
        
        left_front_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_INDEX-LEFT_ANGLE_DEG)
        left_rear_dist = self.get_dist_avg_history(ROBOT_DIRECTION_LEFT_INDEX+LEFT_ANGLE_DEG)

        dx = math.sin(LEFT_ANGLE_RAD) * (left_front_dist - left_rear_dist)
        dy = math.cos(LEFT_ANGLE_RAD) * (left_front_dist + left_rear_dist)

        left_wall_angle = math.atan2(dy, dx)
        #if (left_wall_angle > 0):
        #    left_wall_angle = ((math.pi/180) * 90) - left_wall_angle


def main(args=None):
    rclpy.init(args=args)

    wallfollower = WallFollower()

    rclpy.spin(wallfollower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wallfollower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
