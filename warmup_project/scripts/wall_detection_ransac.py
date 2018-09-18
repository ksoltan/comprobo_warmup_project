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
from visualization_msgs.msg import Marker
from Vector2D import Vector2D
import random
from math import cos, radians

# TODO: create a parent class for Follower which specifies distance/angle to maintain.
class WallDetectionRANSAC(object):
    def __init__(self):
        # init node
        rospy.init_node("wall_detection_node")

        self.distance_to_maintain = 1
        self.angle_to_maintain = 90

        # init subscriber to laser data
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.update_scan_ranges)
        self.scan = None
        self.all_points = None

        # The line of the wall can be described with a slope and y-intercept (y = ax + b)
        # Line in the form ax + by + c = 0
        self.a = 0
        self.b = 0
        self.c = 0

        # Use to store the normal vector to the line (including distance to wall) and the parallel vector to the line
        self.wall_parallel = Vector2D()
        self.wall_normal = Vector2D()

        # init publisher for line visualization
        self.inliers = []
        self.marker_publisher = rospy.Publisher("/visualization_marker",
                                        Marker, queue_size=10)
        # The robot should be a chasing a point, so its target point should be f units further along the wall
        self.f = 1

        # init publisher for target point
        self.target_marker_publisher = rospy.Publisher("/target_marker", Marker, queue_size=10)
        self.target_point_publisher = rospy.Publisher("/target_follow", Twist, queue_size=10)
        self.target_point = Vector2D()

    def update_scan_ranges(self, scan_msg):
        self.scan = scan_msg

    def run(self):
        while not rospy.is_shutdown():
            # Translate polar coordinates from scan to cartesian. Must do in loop
            # instead of in callback to prevent concurrent modification of self.all_points
            self.get_points_from_scan()
            if(self.all_points != None):
                self.ransac()
                # Generate new target point
                self.get_target_point()
                # print("Target point: {}".format(self.target_point))
                marker_target = Marker()
                marker_target.header.frame_id = "base_laser_link"
                marker_target.type = Marker.ARROW
                marker_target.points = [Vector3(0, 0, 0), Vector3(self.target_point.x, self.target_point.y, 0)]
                # marker_target.pose.position.x = self.target_point.x
                # marker_target.pose.position.y = self.target_point.y
                marker_target.scale = Vector3(0.2, 0.5, 0.2)
                marker_target.color.r = 1
                marker_target.color.a = 0.5

                # publish the new target point
                self.target_marker_publisher.publish(marker_target)
                # print(self.target_point)
                twist_msg = Twist()
                twist_msg.linear = Vector3(self.target_point.x, self.target_point.y, 0)
                twist_msg.angular = Vector3(0, 0, self.target_point.angle)
                self.target_point_publisher.publish(twist_msg)

                marker_line = Marker()
                marker_line.header.frame_id = "base_laser_link"
                marker_line.type = Marker.POINTS
                marker_line.scale = Vector3(0.05, 0.05, 0.05)
                marker_line.color.g = 0.9
                marker_line.color.a = 0.5
                marker_line.points = self.inliers
                self.marker_publisher.publish(marker_line)

    def ransac(self):
        # Implement the ransac algorithm
        # Get a list of points representing the scan readings
        best_a = 0
        best_b = 0
        best_c = 0
        max_inliers = 0
        i = 0
        while i < 60:
            # choose two random number of angles to take readings from, fit a line to them, and check how many inliers exist
            self.inliers = [] # Reset the inliers
            a, b, c = self.generate_sample_line()

            num_inliers = self.get_num_inliers(a, b, c)
            # If these settings generate more inliers, use this line.
            if(num_inliers > max_inliers):
                best_a = a
                best_b = b
                best_c = c
                max_inliers = num_inliers
            i += 1
        if(max_inliers > 20):
            print(max_inliers)
            print("Updating Best function")
            self.a = best_a
            self.b = best_b
            self.c = best_c
        # print("Best slope: {} \t Best intercept: {}\t Num inliers: {}".format(self.slope, self.intercept, max_inliers))

    def generate_sample_line(self):
        # Sample two random points
        p, q = self.get_two_random_points()
        a = p.y - q.y
        b = p.x - q.x
        c = p.x * q.y - p.y * q.x
        # print("p = {} \t q = {} \t p-q".format(p, q, p - q))

        return (a, b, c)

    # Calculate least square sums from proposed line for all points
    # Count the number of points that are close enough.
    def get_num_inliers(self, a, b, c):
        inlier_distance = 0.05
        num_inliers = 0
        for p in self.all_points:
            if(p != None and len(self.all_points) >= 2):
                # Find the residual
                residual = 0;
                if b == 0.0: # Line is vertical
                    residual = p.x - c / a
                else:
                    residual = p.y - (a / b * p.x + c / b)
                if(abs(residual) <= inlier_distance):
                    num_inliers += 1
                    self.inliers.append(Point(x=p.x, y=p.y, z=0))
        return num_inliers

    def get_two_random_points(self):
        # Generate two random numbers, keep checking until both ranges are not None
        i1 = random.randint(0, len(self.all_points) - 1)
        i2 = random.randint(0, len(self.all_points) - 1)
        # print("i1 = {}\t, i1 = {}, len all_points = {}".format(i1, i2, len(self.all_points)))
        if(self.all_points[i1] != None and self.all_points[i2] != None and i1 != i2):
            return (self.all_points[i1], self.all_points[i2])
        return self.get_two_random_points()

    def get_points_from_scan(self):
        # Loop through scan and locate points in robot's frame
        # TODO(katya): tf from laser_base to base_link
        if(self.scan != None):
            all_points = []
            for angle in range(0, len(self.scan.ranges)):
                # print("Angle: {}, Range: {}".format(angle, self.scan.ranges[angle]))
                d = self.scan.ranges[angle]
                if(self.is_valid_reading(d)):
                    all_points.append(Vector2D(angle=angle + 180, magnitude=d, unit="deg"))
                    # print("Appended point: {}".format(Vector2D(angle=angle, magnitude=d, unit="deg")))
            self.all_points = all_points

    # Find the point that the robot should attempt to approach (minimize its positional error to)
    # to maintain the angle and distance specified.
    def get_target_point(self):
        if(self.a == 0.0 and self.b == 0.0):
            print("Uh oh...Line not defined")
            return
        if(self.a == 0.0): # Line is horizontal
            normal_distance_to_wall = 1.0 * self.c / self.b
            v_parallel = Vector2D(x=0.0, y=normal_distance_to_wall)
            v_normal = v_parallel.normal()
            # print("Horiz")
        elif(self.b == 0.0): # Line if vertical
            normal_distance_to_wall = 1.0 * self.c
            v_parallel = Vector2D(x=normal_distance_to_wall, y=0.0)
            v_normal = v_parallel.normal()
            # print("Vert")
        else:
            # Get the angle of the normal to the line
            v_parallel = Vector2D(x=1, y=1.0 * self.a / self.b)
            v_normal = v_parallel.normal().hat()
            normal_distance_to_wall = 1.0 * self.c / self.b * cos(radians(90) - v_normal.angle)
            v_normal = v_normal * normal_distance_to_wall
            # print("Reg")
        self.target_point = v_parallel.hat() * self.f + v_normal.hat() * (v_normal.magnitude - self.distance_to_maintain)
        self.target_point.rotate(90)

    def is_valid_reading(self, reading):
        return self.scan.range_min <= reading <= self.scan.range_max

    def get_all_points(self):
        l = []
        for p in self.all_points:
            l.append(Point(x=p.x, y=p.y, z=0))
        return l


if __name__ == "__main__":
    node = WallDetectionRANSAC()
    node.run()
