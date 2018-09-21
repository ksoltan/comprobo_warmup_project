#!/usr/bin/env python
"""
Given a target object to follow, maintains a specific distance relative to that object.
This node subscribes to a target publisher and gives back a velocity correction.
"""

from __future__ import print_function

from geometry_msgs.msg import Twist, Vector3, PointStamped
from warmup_project.msg import State, LabeledPolarVelocity2D, PolarVelocity2D
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from math import sin, cos, sqrt, radians, degrees, pi, atan2
import math
import rospy
import tf

class MaintainDistance(object):
    def __init__(self):
        behavior = "maintain_distance"
        rospy.init_node(behavior + "_node")
        self.behavior_id = State.MAINTAIN_DISTANCE

        # Set the distance to maintain relative to a target.
        # The default is to be at the target
        # self.distance = rospy.get_param('distance', 0)
        self.angle = 0
        self.k_p = 0.7
        self.max_linear_speed = 0.1
        self.max_angular_speed = 0.5
        self.target_pos = None
        # Setup subscriber that defines the target. The target is at a position relative to
        # the robot (aka in the base_link frame) specified by a Twist linear and angular position.
        self.target_subscriber = rospy.Subscriber("/target_follow", PointStamped, self.update_target)

        self.cmd_vel_publisher = rospy.Publisher("/desired_cmd_vel", LabeledPolarVelocity2D, queue_size=10)
        self.target_marker_publisher = rospy.Publisher("/target_marker", Marker, queue_size=10)
        self.tf_listener = tf.TransformListener()
        print("Hi")

    def update_target(self, target_msg):
        self.target_pos = target_msg
        # print(self.target_pos)

        # print("My target = {}".format(target_msg))

    '''update_postion: Calculated the error between the target and the current pose of the robot.
    Sends an updated cmd_vel to lessen the error.'''
    def update_position(self):
        # Transform from base_laser_link to base_link
        tf_target = self.tf_listener.transformPoint('base_link', self.target_pos)
        # tf_target = self.target_pos
        # print(tf_target)
        # If the angle is not aligned, turn. Otherwise, go forward.
        error = sqrt(tf_target.point.x**2 + tf_target.point.y**2)
        # print("x, y = {}, {}\terror = {}".format(self.target_pos.linear.x, self.target_pos.linear.y, error))
        # new_cmd_vel = self.k_p * error * cos(self.target_pos.angular.z)

        # angular_error = self.target_pos.angular.z
        # angular_error = error * cos(self.target_pos.angular.z)
        # angular_error = atan2(tf_target.point.y, tf_target.point.x) % (math.pi * 2)
        # angular_error = atan2(self.target_pos.point.y, self.target_pos.point.x)
        # angular_error = self.target_pos.point.z

        if(abs(tf_target.point.y) < 0.3):
            # Allow some forward speed
            new_cmd_vel = self.max_linear_speed
            new_cmd_angle = 0
        else:
            new_cmd_vel = 0
            new_cmd_angle = self.max_angular_speed

        # Project the vector to the target point onto the forward unit vector of the robot.
        # This way, if the target point is behind or in a different direction, more weight will
        # be given to the angular compensation and the robot will slow down.

        # error = sqrt(self.target_pos.linear.x**2 + self.target_pos.linear.y**2) - self.distance
        # print("x, y = {}, {}\terror = {}".format(self.target_pos.linear.x, self.target_pos.linear.y, error))
        # new_cmd_vel = self.k_p * error * cos(self.target_pos.angular.z)
        # new_cmd_angle = 1.0 * self.k_p * self.target_pos.angular.z * self.max_angular_speed
        print("new_cmd_vel = {}, new_cmd_angle = {}".format(round(new_cmd_vel, 2), round(new_cmd_angle, 2)))
        # print("angular_error = {}, laser_link angle = {}".format(round(degrees(angular_error), 2),round(degrees(self.target_pos.point.z))))
        print("x_component = {}, y_component = {}".format(tf_target.point.x, tf_target.point.y))

        marker_target = Marker()
        marker_target.header.frame_id = "base_laser_link"
        marker_target.type = Marker.ARROW
        marker_target.points = [Vector3(0, 0, 0), Vector3(self.target_pos.point.x, self.target_pos.point.y, 0)]
        marker_target.scale = Vector3(0.1, 0.4, 0.1)
        marker_target.color.r = 1
        marker_target.color.a = 0.5

        # publish the new target point
        self.target_marker_publisher.publish(marker_target)
        new_polar_vel = LabeledPolarVelocity2D(node_ID=State.FOLLOW, velocity=PolarVelocity2D(linear=new_cmd_vel, angular=new_cmd_angle))

        # TODO: should I be publishing to a different place?
        self.cmd_vel_publisher.publish(new_polar_vel)

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/base_laser_link', rospy.Time(0))
                # print("Got the transform")
                if(self.target_pos != None):
                    self.update_position()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


            # r.sleep() # Calculate error every 10 seconds.

if __name__ == "__main__":
    node = MaintainDistance()
    node.run()
