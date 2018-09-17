#!/usr/bin/env python
"""
Given a target object to follow, maintains a specific angle relative to that object.
This node subscribes to a target publisher and gives back an angular correction.
"""

from __future__ import print_function

from geometry_msgs.msg import Twist, Vector3
from warmup_project.msg import State, LabeledPolarVelocity2D, PolarVelocity2D
from sensor_msgs.msg import LaserScan
from math import sin, cos, sqrt, radians
import rospy

class MaintainAngle(object):
    def __init__(self):
        behavior = "maintain_angle"
        rospy.init_node(behavior + "_node")
        self.behavior_id = State.MAINTAIN_ANGLE

        # Set the angle to maintain relative to a target.
        # The default is to keep the object in front of the robot.
        self.angle = rospy.get_param('angle', 0)
        self.k_p = 0.5
        self.max_angular_speed = 2
        self.target_pos = Twist()

        # Setup subscriber that defines the target. The target is at a position relative to
        # the robot (aka in the base_link frame) specified by a Twist linear and angular position.
        self.target_subscriber = rospy.Subscriber("/target_follow", Twist, self.update_target)

        # TODO: setup publisher
        self.cmd_vel_publisher = rospy.Publisher("/desired_cmd_vel", LabeledPolarVelocity2D, queue_size=10)

    def update_target(self, target_msg):
        self.target_pos = target_msg
        print(target_msg)

    '''update_postion: Calculated the error between the target and the current pose of the robot.
    Sends an updated cmd_vel to lessen the error.'''
    def update_position(self):
        # The robot is positioned at (0, 0). If the angle of the target is no self.angle,
        # there is an error.
        error = self.angle - self.target_pos.angular.z
        print("error is: {}".format(error))
        # If the error is 0, the robot is at the correct angle. The new_cmd_angle should be 0.
        # As the error gets smaller, the slower the angular speed becomes.
        new_cmd_angle = min(abs(self.k_p * error * self.max_angular_speed), self.max_angular_speed)
        new_polar_vel = LabeledPolarVelocity2D(node_ID=self.behavior_id, velocity=PolarVelocity2D(linear=0, angular=new_cmd_angle))

        print("I want to turn this much: {}".format(new_cmd_angle))
        # TODO: should I be publishing to a different place?
        self.cmd_vel_publisher.publish(new_polar_vel)

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update_position()
            r.sleep() # Calculate error every 10 seconds.

if __name__ == "__main__":
    node = MaintainAngle()
    node.run()
