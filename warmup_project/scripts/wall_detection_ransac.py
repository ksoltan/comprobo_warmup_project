#!/usr/bin/env python
"""
This is a RANSAC implementation of detecting a wall using the laser scan of the Neato.
When it detects the wall, it calculates what point the robot should be aiming towards
to achieve a certain specified distance and angle to the wall.
"""
from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist, Vector3, Point
from warmup_project.msg import State, LabeledPolarVelocity2D, PolarVelocity2D
from sensor_msgs.msg import LaserScan
from Vector2D import Vector2D
import random

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
        self.all_points = []

        # The line of the wall can be described with a slope and y-intercept (y = ax + b)
        self.slope = 0
        self.intercept = 0
        # init publisher for line visualization
        self.inliers = []
        self.marker_publisher = rospy.Publisher("/visualization_marker",
                                        Marker, queue_size=10)
        # The robot should be a chasing a point, so its target point should be f units further along the wall
        self.f = 1

        # init publisher for target point
        self.target_point_publisher = rospy.Publisher("/target_point", Point, queue_size=10)
        self.target_point = Vector2D()

    def update_scan_ranges(self, scan_msg):
        self.scan_ranges = scan_msg.ranges
        self.get_points_from_scan()

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Update predicted line
            self.ransac()
            # Generate new target point
            self.get_target_point()
            # publish the new target point
            self.target_point_publisher.publish(Point(self.target_point.x, self.target_point.y, 0))
            #TODO: publish visualization of the line for debugging!
            marker_line = Marker()
            marker_line.header.frame_id = "base_link"
            marker_line.type = Marker.LINE_LIST
            marker_line.scale = Vector3(1, 1, 1)
            marker_line.color.g = 0.9
            marker_line.color.a = 0.5
            marker_line.points = self.inliers
            r.sleep() # Calculate error every 10 seconds.

    def ransac(self):
        # Implement the ransac algorithm
        # Get a list of points representing the scan readings
        best_slope = 0
        best_intercept = 0
        max_inliers = 0
        i = 0
        while i < 25:
            # choose two random number of angles to take readings from, fit a line to them, and check how many inliers exist
            p1, p2 = self.get_two_random_points()
            slope = (p1.y - p2.y) * 1.0 / (p1.x - p2.x)
            intercept = -1.0 * slope * p1.x + p1.y

            # If these settings generate more inliers, use this line.
            if(self.get_num_inliers(slope, intercept) > max_inliers):
                best_slope = slope
                best_intercept = intercept
        self.slope = best_slope
        self.intercept = best_intercept

    # Calculate least square sums from proposed line for all points
    # Count the number of points that are close enough.
    def get_num_inliers(self, slope, intercept):
        inlier_distance = 0.1
        num_inliers = 0
        for p in all_points:
            if(p != None):
                # Find the residual
                residual = p.y - (slope * p.x + intercept)
                if(abs(residual) <= inlier_distance):
                    num_inliers += 1
                    self.inliers.append(Point(x=p.x, y=p.y, 0))
        return num_inliers

    def get_two_random_points(self):
        # Generate two random numbers, keep checking until both ranges are not None
        i1 = random.randint(0, len(self.scan_ranges) - 1)
        i2 = random.randint(0, len(self.scan_ranges) - 1)
        if(all_points[i1] != None and all_points[i2] != None):
            return (all_points[i1], all_points[i2])
        return self.get_two_random_points()

    def get_points_from_scan(self):
        # Loop through scan and locate points in robot's frame
        # TODO(katya): tf from laser_base to base_link
        all_points = []
        for angle in range(0, len(self.scan_ranges)):
            range = self.scan_ranges[angle]
            if(range == 0.0):
                all_points.append(None)
            else:
                all_points.append(Vector2D(angle=angle, magnitude=range))
        self.all_points = all_points

    # Find the point that the robot should attempt to approach (minimize its positional error to)
    # to maintain the angle and distance specified.
    def get_target_point(self):
        # Define the vector (direction) of the detected line
        parallel_vector = Vector2D(x=1, y=self.slope)
        perp_vector = Vector2D(x=1, y=-self.slope)
        perp_vector_angle = perp_vector.angle()
        # Calculate the displacement along the vector normal to the wall from the robot
        # to maintain the distance from it
        normal_distance_to_wall = self.intercept * cos(90 - perp_vector_angle)

        target_parallel_displacement = parallel_vector.hat() * self.f
        target_perp_displacement = perp_vector.hat() * (normal_distance_to_wall - self.distance_to_maintain)
        self.target_point = target_parallel_displacement + target_perp_displacement
