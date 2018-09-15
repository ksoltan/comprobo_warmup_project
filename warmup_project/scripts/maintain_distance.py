#!/usr/bin/env python
"""
Given a target object to follow, maintains a specific distance relative to that object.
This node subscribes to a target publisher and gives back a velocity correction.
"""

from __future__ import print_function

from geometry_msgs.msg import Twist, Vector3
from warmup_project.msg import State, LabeledPolarVelocity2D, PolarVelocity2D
from sensor_msgs.msg import LaserScan
from math import sin, cos, sqrt, radians
import rospy

class MaintainDistance(object):
    def __init__(self):
        behavior = "maintain_distance"
        rospy.init_node(behavior + "_node")
        self.behavior_id = State.MAINTAIN_DISTANCE

        # Set the distance to maintain relative to a target.
        # The default is to keep 1 m behind the object
        self.distance = rospy.get_param('distance', 1)
        self.k_p = 0.5
        self.max_linear_speed = 2
        self.angular_speed = 1
        self.target_pos = Twist()

        # Setup subscriber that defines the target. The target is at a position relative to
        # the robot (aka in the base_link frame) specified by a Twist linear and angular position.
        self.target_subscriber = rospy.Subscriber("/target_follow", Twist, self.update_target)

        self.cmd_vel_publisher = rospy.Publisher("/desired_cmd_vel", LabeledPolarVelocity2D, queue_size=10)

    def update_target(self, target_msg):
        self.target_pos = target_msg

    '''update_postion: Calculated the error between the target and the current pose of the robot.
    Sends an updated cmd_vel to lessen the error.'''
    def update_position(self):
        error = sqrt(self.target_pos.linear.x**2 + self.target_poslinear.y**2) - self.distance
        # If the error is 0, the robot is at the correct distance. The new_cmd_vel should be 0
        # The farther the robot is from the object (positive error), the higher the linear speed should be.
        # If the error is negative, the robot is too close and needs to move back.
        # The robot should be moving along the vector to its object. Therefore, there is an angular speed.
        new_cmd_vel = min(abs(self.k_p * error * self.max_linear_speed), self.max_linear_speed)
        new_cmd_angle = min(abs(self.k_p * error * self.max_angular_speed), self.max_angular_speed)
        new_polar_vel = LabeledPolarVelocity2D(node_ID=self.behavior_id, velocity=PolarVelocity2D(linear=new_cmd_vel, angular=new_cmd_angle))

        # TODO: should I be publishing to a different place?
        cmd_vel_publisher.publish(new_polar_vel)

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update_position()
            r.sleep() # Calculate error every 10 seconds.

if __name__ == "__main__":
    node = MaintainDistance()
    node.run()
