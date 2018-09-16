#!/usr/bin/env python
"""
This is a RANSAC implementation of detecting a wall using the laser scan of the Neato.
When it detects the wall, it calculates what point the robot should be aiming towards
to achieve a certain specified distance and angle to the wall.
"""
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist, Vector3
from warmup_project.msg import State, LabeledPolarVelocity2D, PolarVelocity2D
from sensor_msgs.msg import LaserScan
from math import sin, cos, sqrt, radians

# TODO: create a parent class for Follower which specifies distance/angle to maintain.
class WallDetectionRANSAC(object):
    def __init__(self):
        # init node
        rospy.init_node("wall_detection_node")

        self.distance_to_maintain = 1
        self.angle_to_maintain = 90

        # init subscriber to laser data
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.update_scan_ranges)
        self.scan_ranges = []

        # The line of the wall can be described with a slope and y-intercept (y = ax + b)
        self.slope = 0
        self.intercept = 0

        # The robot should be a chasing a point, so its target point should be f units further along the wall
        self.f = 1

    def update_scan_ranges(self, scan_msg):
        self.scan_ranges = scan_msg.ranges

    def find_wall(self):


    # Find the point that the robot should attempt to approach (minimize its positional error to)
    # to maintain the angle and distance specified.
    def get_target_point(self):
        # Calculate the displacement along the vector normal to the wall from the robot
        # to maintain the distance from it
        
        normal_distance_to_wall =
        along_wall = Vector3()
